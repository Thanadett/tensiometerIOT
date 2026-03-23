/**
 * ============================================================
 *  Tensiometer — ESP32 + LoRa (SX1276) + MPX5050 + VDiv
 * ============================================================
 *
 *  Hardware Wiring
 *  ───────────────
 *  MPX5050 (Vcc=5V, Vout → Voltage Divider → ESP32 ADC)
 *    MPX5050 Pin 1 (Vout) ── R1(10kΩ) ──┬── R2(10kΩ) ── GND
 *                                         └── GPIO34 (ADC1_CH6)
 *    MPX5050 Pin 2 (GND)  ── GND
 *    MPX5050 Pin 3 (Vcc)  ── 5V
 *
 *  Voltage Divider: Vout_max(MPX5050)=4.7V → ADC ~2.35V (safe for 3.3V ADC)
 *  Divider ratio = R2/(R1+R2) = 0.5  → multiply ADC reading × 2
 *
 *  LoRa SX1276 (3.3V)
 *    NSS  ── GPIO5
 *    MOSI ── GPIO23
 *    MISO ── GPIO19
 *    SCK  ── GPIO18
 *    RST  ── GPIO14
 *    DIO0 ── GPIO26
 *
 *  MPX5050 Specs
 *  ─────────────
 *  Range     :  0 – 50 kPa (measures vacuum magnitude from tensiometer)
 *  Vout      :  0.2V (0 kPa) – 4.7V (50 kPa)
 *  Supply    :  5V ± 0.25V
 *  Formula   :  raw_kPa = (Vout - Vmin - ZERO_OFFSET) / (Vmax - Vmin) * 50
 *  Sign      :  tensiometer = vacuum → soil_tension = -raw_kPa  (negative pressure)
 *              → report as positive cbar: soil_tension_cbar = abs(raw_kPa)
 *
 *  Calibration
 *  ───────────
 *  Each MPX5050 has a unique zero-offset. Measure Vout with sensor open
 *  to atmosphere (0 kPa), then set ZERO_OFFSET = Vout_measured - 0.20
 *  Example: if Vout@0kPa = 0.23V → ZERO_OFFSET = 0.03f
 *  This reduces error from ±5 kPa → ±1 kPa
 *
 *  Soil Tension Interpretation (centibars = kPa)
 *  ──────────────────────────────────────────────
 *  0  –  10 cbar : Saturated / waterlogged
 *  10 – 30 cbar  : Field capacity (ideal after irrigation)
 *  30 – 50 cbar  : Optimal range for most crops
 *  > 50 cbar     : Severe stress — irrigate immediately
 * ============================================================
 */

#include <Arduino.h>
#include <ArduinoJson.h> // https://arduinojson.org/

// ─── LoRa Enable ────────────────────────────────────────────
// 1 = LoRa module plugged in → transmit JSON every LORA_SEND_INTERVAL
// 0 = no module → sensor-only mode, Serial output only
#define ENABLE_LORA 1

#if ENABLE_LORA
#include <SPI.h>
#include <LoRa.h> // https://github.com/sandeepmistry/arduino-LoRa
#endif

// ─── Pin Definitions ────────────────────────────────────────
#define ADC_PIN 34 // GPIO34 — ADC1_CH6 (input-only, no pull-up)
#define LORA_NSS 5
#define LORA_RST 14
#define LORA_DIO0 26

// ─── ADC & Voltage Divider ──────────────────────────────────
// analogReadMilliVolts() used — no manual ADC_MAX/VREF needed
#define ADC_ATTEN ADC_11db // allows 0–3.6V input range
#define VDIV_RATIO 2.0f    // × 2 to recover original Vout from divider

// ─── MPX5050 Transfer Function ──────────────────────────────
#define MPX_VOUT_MIN 0.20f // V at 0 kPa (nominal)
#define MPX_VOUT_MAX 4.70f // V at 50 kPa
#define MPX_P_MAX 50.0f    // kPa full scale

// ─── Sensor Calibration ─────────────────────────────────────
// HOW TO CALIBRATE:
//   1. Disconnect porous cup — leave sensor open to atmosphere (0 kPa)
//   2. Read Vout from Serial: "[SENS] Vout=X.XXXV"
//   3. ZERO_OFFSET = Vout_measured - 0.20
//
// Your sensor measured Vout=0.284V at ~0 kPa:
//   ZERO_OFFSET = 0.284 - 0.20 = 0.084V  ← set below
//
// Note: analogReadMilliVolts() already includes ADC nonlinearity correction,
// so Vout here is the actual voltage AFTER the voltage divider × 2.
#define ZERO_OFFSET 0.084f // V  ← derived from your Vout=0.284V reading

// ─── Filtering Parameters ───────────────────────────────────
#define SAMPLE_N 16        // Samples per reading (median filter window)
#define EMA_ALPHA 0.15f    // EMA smoothing factor (lower = smoother, slower)
#define OUTLIER_SIGMA 2.5f // Reject samples > N×σ from mean

// ─── Timing ─────────────────────────────────────────────────
#define SAMPLE_INTERVAL_MS 5000  // Read sensor every 5 s
#define LORA_SEND_INTERVAL 60000 // Transmit via LoRa every 60 s

// ─── LoRa Settings ──────────────────────────────────────────
#define LORA_FREQUENCY 433E6
#define LORA_SF 10       // Spreading Factor (7–12)
#define LORA_BW 125E3    // Bandwidth (Hz)
#define LORA_CR 5        // Coding Rate (5=4/5)
#define LORA_TX_POWER 17 // dBm

// ─── Device Identity ────────────────────────────────────────
#define DEVICE_ID "TENS-01"

// ════════════════════════════════════════════════════════════
//  Globals
// ════════════════════════════════════════════════════════════
static float g_ema_pressure = NAN; // EMA accumulator (NAN = uninitialised)
static float g_last_cbar = 0.0f;
static uint32_t g_last_sample_ms = 0;
#if ENABLE_LORA
static uint32_t g_last_lora_ms = 0;
static uint32_t g_packet_count = 0;
#endif

// ════════════════════════════════════════════════════════════
//  Utility: Insertion Sort (in-place, ascending)
// ════════════════════════════════════════════════════════════
static void insertion_sort(float *arr, uint8_t n)
{
  for (uint8_t i = 1; i < n; i++)
  {
    float key = arr[i];
    int8_t j = i - 1;
    while (j >= 0 && arr[j] > key)
    {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

// ════════════════════════════════════════════════════════════
//  ADC → Voltage (with multi-sample median + outlier rejection)
// ════════════════════════════════════════════════════════════
static float read_adc_voltage()
{
  float samples[SAMPLE_N];

  // 1. Collect samples — use analogReadMilliVolts() for better ESP32 ADC linearity
  //    (~3× more accurate than raw analogRead due to internal nonlinearity correction)
  for (uint8_t i = 0; i < SAMPLE_N; i++)
  {
    float mv = (float)analogReadMilliVolts(ADC_PIN); // mV, already linearised
    samples[i] = mv / 1000.0f;                       // convert to V
    delayMicroseconds(500);
  }

  // 2. Compute mean and std-dev for outlier detection
  float sum = 0.0f;
  for (uint8_t i = 0; i < SAMPLE_N; i++)
    sum += samples[i];
  float mean = sum / SAMPLE_N;

  float var = 0.0f;
  for (uint8_t i = 0; i < SAMPLE_N; i++)
  {
    float d = samples[i] - mean;
    var += d * d;
  }
  float sigma = sqrtf(var / SAMPLE_N);

  // 3. Reject outliers (replace with mean)
  uint8_t valid_n = 0;
  float valid_sum = 0.0f;
  for (uint8_t i = 0; i < SAMPLE_N; i++)
  {
    if (fabsf(samples[i] - mean) <= OUTLIER_SIGMA * sigma)
    {
      valid_sum += samples[i];
      valid_n++;
    }
  }

  // 4. Sort remaining valid samples → median
  float valid[SAMPLE_N];
  uint8_t idx = 0;
  for (uint8_t i = 0; i < SAMPLE_N; i++)
  {
    if (fabsf(samples[i] - mean) <= OUTLIER_SIGMA * sigma)
    {
      valid[idx++] = samples[i];
    }
  }

  if (idx == 0)
    return mean * VDIV_RATIO; // fallback

  insertion_sort(valid, idx);
  float median = (idx % 2 == 0)
                     ? (valid[idx / 2 - 1] + valid[idx / 2]) * 0.5f
                     : valid[idx / 2];

  // 5. Restore original Vout (undo voltage divider)
  return median * VDIV_RATIO;
}

// ════════════════════════════════════════════════════════════
//  Vout → Soil Tension (kPa)
//
//  MPX5050 measures vacuum magnitude (positive output).
//  Tensiometer physics: soil tension = NEGATIVE pressure.
//
//  Steps:
//    1. Apply zero-offset calibration
//    2. Map Vout → raw_kPa (0..50, positive)
//    3. Invert sign → negative pressure (vacuum)
//    4. Caller uses fabsf() to get soil tension in cbar
// ════════════════════════════════════════════════════════════
static float voltage_to_kpa(float vout)
{
  // 1. Calibrated Vout (subtract sensor-specific zero-offset)
  float vout_cal = vout - ZERO_OFFSET;

  // 2. MPX5050 transfer function → positive vacuum magnitude
  float raw_kpa = (vout_cal - MPX_VOUT_MIN) / (MPX_VOUT_MAX - MPX_VOUT_MIN) * MPX_P_MAX;
  raw_kpa = constrain(raw_kpa, 0.0f, MPX_P_MAX);

  // 3. Tensiometer = vacuum → negative pressure
  return -raw_kpa;
}

// ════════════════════════════════════════════════════════════
//  EMA (Exponential Moving Average) filter
//  y[n] = α·x[n] + (1-α)·y[n-1]
//  Input: soil_tension (positive cbar)
// ════════════════════════════════════════════════════════════
static float ema_update(float new_val)
{
  if (isnan(g_ema_pressure))
  {
    g_ema_pressure = new_val; // seed on first call
  }
  else
  {
    g_ema_pressure = EMA_ALPHA * new_val + (1.0f - EMA_ALPHA) * g_ema_pressure;
  }
  return g_ema_pressure;
}

// ════════════════════════════════════════════════════════════
//  Soil status string from centibars
// ════════════════════════════════════════════════════════════
static const char *soil_status(float cbar)
{
  if (cbar < 10)
    return "SATURATED";
  if (cbar < 30)
    return "FIELD_CAPACITY";
  if (cbar < 50)
    return "OPTIMAL";
  return "IRRIGATE_NOW";
}

// ════════════════════════════════════════════════════════════
//  LoRa Transmit (JSON payload) — compiled only when ENABLE_LORA=1
// ════════════════════════════════════════════════════════════
#if ENABLE_LORA
static bool lora_send(float cbar, float vout, float kpa)
{
  StaticJsonDocument<256> doc;
  doc["id"] = DEVICE_ID;
  doc["ts"] = millis() / 1000UL; // seconds uptime
  doc["pkt"] = ++g_packet_count;
  doc["cbar"] = serialized(String(cbar, 2));
  doc["kpa"] = serialized(String(kpa, 2));
  doc["vout"] = serialized(String(vout, 3));
  doc["status"] = soil_status(cbar);

  char buf[256];
  size_t len = serializeJson(doc, buf);

  LoRa.beginPacket();
  LoRa.print(buf);
  bool ok = LoRa.endPacket();

  Serial.printf("[LoRa] %s → %s\r\n", ok ? "TX OK" : "TX FAIL", buf);
  Serial.flush();
  return ok;
}

#endif // ENABLE_LORA

// ════════════════════════════════════════════════════════════
//  Setup
// ════════════════════════════════════════════════════════════
void setup()
{
  Serial.begin(115200);
  delay(1000); // wait for USB-Serial to stabilise on host side
  Serial.flush();
  Serial.println("\r\n=== Tensiometer ESP32 Boot ===");
  Serial.flush();

  // ── ADC ───────────────────────────────────────────────────
  // Configure BEFORE SPI/LoRa so ADC mux does not interfere with SPI bus
  analogSetAttenuation(ADC_ATTEN);
  analogSetPinAttenuation(ADC_PIN, ADC_ATTEN);
  Serial.printf("[ADC] Pin %d configured, 11dB atten, using analogReadMilliVolts()\r\n",
                ADC_PIN);
  Serial.flush();
  delay(100); // settle ADC before touching SPI

#if ENABLE_LORA
  // ── LoRa ────────────────────────────────────────────────
  SPI.begin();
  delay(100);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

  bool lora_ok = false;
  for (uint8_t attempt = 1; attempt <= 5; attempt++)
  {
    Serial.printf("[LoRa] Init attempt %d/5...\r\n", attempt);
    Serial.flush();
    if (LoRa.begin(LORA_FREQUENCY))
    {
      lora_ok = true;
      break;
    }
    delay(500);
  }
  if (!lora_ok)
  {
    Serial.println("[LoRa] INIT FAILED — check wiring!\r\n"
                   "       NSS=5 MOSI=23 MISO=19 SCK=18 RST=14 DIO0=26");
    Serial.flush();
    while (true)
      delay(1000);
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setSyncWord(0xF3);
  Serial.printf("[LoRa] Ready @ %.0f MHz  SF%d  BW%.0fkHz\r\n",
                LORA_FREQUENCY / 1e6, LORA_SF, LORA_BW / 1e3);
  Serial.flush();
#else
  Serial.println("[LoRa] Disabled (ENABLE_LORA=0) — Serial output only\r\n");
  Serial.flush();
#endif // ENABLE_LORA

  Serial.println("[SYS] Entering main loop...\r\n");
  Serial.flush();
}

// ════════════════════════════════════════════════════════════
//  Main Loop
// ════════════════════════════════════════════════════════════
void loop()
{
  uint32_t now = millis();

  // ── Sensor sampling ─────────────────────────────────────
  if (now - g_last_sample_ms >= SAMPLE_INTERVAL_MS)
  {
    g_last_sample_ms = now;

    // 1. Read & filter ADC → Vout
    float vout = read_adc_voltage();

    // 2. Convert to pressure — returns NEGATIVE kPa (vacuum)
    float kpa = voltage_to_kpa(vout);

    // 3. Soil tension = absolute value (positive cbar)
    //    e.g. kpa=-25.0 → tension=25.0 cbar
    float tension = fabsf(kpa);

    // 4. EMA smoothing on soil tension (positive cbar)
    float cbar = ema_update(tension);
    g_last_cbar = cbar;

    // 5. Serial debug — \r\n for compatibility with all serial monitors
    Serial.printf("[SENS] Vout=%.3fV  Vacuum=%.2fkPa  Tension=%.2fcbar  EMA=%.2fcbar  [%s]\r\n",
                  vout, kpa, tension, cbar, soil_status(cbar));
    Serial.flush();
  }

#if ENABLE_LORA
  // ── LoRa transmit ──────────────────────────────────────
  if (now - g_last_lora_ms >= LORA_SEND_INTERVAL)
  {
    g_last_lora_ms = now;
    float vout = read_adc_voltage();
    float kpa = voltage_to_kpa(vout);
    float tension = fabsf(kpa);
    lora_send(g_last_cbar, vout, tension);
    delay(50);
  }
#endif // ENABLE_LORA

  // ── Small yield delay ───────────────────────────────────
  delay(10);
}