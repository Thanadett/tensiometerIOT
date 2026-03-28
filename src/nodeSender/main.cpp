#include <Arduino.h>
#include <ArduinoJson.h>
#include <esp_sleep.h>

#define ENABLE_LORA 1

#if ENABLE_LORA
#include <SPI.h>
#include <LoRa.h>
#endif

// ─── Pin Definitions ────────────────────────────────────────
#define ADC_PIN_S1 34 // Sensor 1 (ความลึก 40 cm)
#define ADC_PIN_S2 35 // Sensor 2 (ความลึก 60 cm)
#define LORA_NSS 5
#define LORA_RST 14
#define LORA_DIO0 26

// ─── ADC ─────────────────────────────────────────────────────
#define ADC_ATTEN ADC_11db // 0–3.6V range
#define VDIV_RATIO 1.50f   // Vout voltage divider

// ─── MPX5050 Transfer Function ──────────────────────────────
#define MPX_VOUT_MIN 0.20f // V ที่ 0 kPa
#define MPX_VOUT_MAX 4.70f // V ที่ 50 kPa
#define MPX_P_MAX 50.0f    // kPa full scale

// ─── Calibration (ตั้งค่าแยกสำหรับแต่ละตัว) ────────────────
#define ZERO_OFFSET_S1 0.013f // ← sensor ตัวที่ 1
#define ZERO_OFFSET_S2 0.033f // ← sensor ตัวที่ 2

// ─── ความลึกของ Sensor ──────────────────────────────────────
#define DEPTH_CM_S1 40 // cm
#define DEPTH_CM_S2 60 // cm

// ─── Filtering ──────────────────────────────────────────────
#define SAMPLE_N 16        // จำนวน sample ต่อการอ่าน
#define EMA_ALPHA 0.15f    // EMA factor
#define OUTLIER_SIGMA 2.5f // ตัดค่า outlier เกิน N×σ

// ─── Timing ─────────────────────────────────────────────────
#define SLEEP_DURATION_S 300ULL // วินาที (5 นาที)

// ─── LoRa Settings ──────────────────────────────────────────
#define LORA_FREQUENCY 433E6
#define LORA_SF 10
#define LORA_BW 125E3
#define LORA_CR 5
#define LORA_TX_POWER 17
#define LORA_SYNC_WORD 0xF3

// ─── Device Identity ────────────────────────────────────────
#define DEVICE_ID "TENS-01"

// ════════════════════════════════════════════════════════════
//  RTC Memory
// ════════════════════════════════════════════════════════════
RTC_DATA_ATTR static uint32_t g_packet_count = 0; // นับ packet ทั้งหมด
RTC_DATA_ATTR static uint32_t g_boot_count = 0;   // นับครั้งที่ตื่น
RTC_DATA_ATTR static float g_ema_s1 = NAN;        // EMA sensor 1
RTC_DATA_ATTR static float g_ema_s2 = NAN;        // EMA sensor 2

// ════════════════════════════════════════════════════════════
//  Struct: ผลลัพธ์ต่อ sensor หนึ่งตัว
// ════════════════════════════════════════════════════════════
struct SensorReading
{
  float vout;    // V  (หลัง voltage divider × 2)
  float kpa;     // kPa (negative = vacuum)
  float tension; // cbar (positive, = fabsf(kpa))
  float cbar;    // cbar หลัง EMA smooth
};

// ════════════════════════════════════════════════════════════
//  Utility: Insertion Sort
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
//  ADC → Vout  (median + outlier rejection)
// ════════════════════════════════════════════════════════════
static float read_adc_voltage(uint8_t pin)
{
  float samples[SAMPLE_N];

  // เก็บ sample
  for (uint8_t i = 0; i < SAMPLE_N; i++)
  {
    samples[i] = (float)analogReadMilliVolts(pin) / 1000.0f;
    delayMicroseconds(500);
  }

  // mean + σ
  float sum = 0;
  for (uint8_t i = 0; i < SAMPLE_N; i++)
    sum += samples[i];
  float mean = sum / SAMPLE_N;

  float var = 0;
  for (uint8_t i = 0; i < SAMPLE_N; i++)
  {
    float d = samples[i] - mean;
    var += d * d;
  }
  float sigma = sqrtf(var / SAMPLE_N);

  // กรอง outlier + median
  float valid[SAMPLE_N];
  uint8_t idx = 0;
  for (uint8_t i = 0; i < SAMPLE_N; i++)
  {
    if (fabsf(samples[i] - mean) <= OUTLIER_SIGMA * sigma)
      valid[idx++] = samples[i];
  }
  if (idx == 0)
    return mean * VDIV_RATIO;

  insertion_sort(valid, idx);
  float median = (idx % 2 == 0)
                     ? (valid[idx / 2 - 1] + valid[idx / 2]) * 0.5f
                     : valid[idx / 2];

  return median * VDIV_RATIO; // recover Vout จริง
}

// ════════════════════════════════════════════════════════════
//  Vout → kPa  (negative = vacuum = soil tension)
// ════════════════════════════════════════════════════════════
static float voltage_to_kpa(float vout, float zero_offset)
{
  float vout_cal = vout - zero_offset;
  float raw_kpa = (vout_cal - MPX_VOUT_MIN) / (MPX_VOUT_MAX - MPX_VOUT_MIN) * MPX_P_MAX;
  raw_kpa = constrain(raw_kpa, 0.0f, MPX_P_MAX);
  return -raw_kpa; // negative = vacuum
}

// ════════════════════════════════════════════════════════════
//  EMA Update  (ใช้ pointer เพื่อแก้ไข g_ema_s1 / g_ema_s2 ใน RTC RAM)
// ════════════════════════════════════════════════════════════
static float ema_update(float *ema_state, float new_val)
{
  if (isnan(*ema_state))
    *ema_state = new_val;
  else
    *ema_state = EMA_ALPHA * new_val + (1.0f - EMA_ALPHA) * (*ema_state);
  return *ema_state;
}

// ════════════════════════════════════════════════════════════
//  Soil Status String
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
//  อ่าน Sensor ทั้งคู่ — return ผ่าน struct
// ════════════════════════════════════════════════════════════
static SensorReading read_sensor(uint8_t pin, float zero_offset, float *ema_state)
{
  SensorReading r;
  r.vout = read_adc_voltage(pin);
  r.kpa = voltage_to_kpa(r.vout, zero_offset);
  r.tension = fabsf(r.kpa);
  r.cbar = ema_update(ema_state, r.tension);
  return r;
}

// ════════════════════════════════════════════════════════════
//  LoRa Init
// ════════════════════════════════════════════════════════════
#if ENABLE_LORA
static bool lora_init()
{
  SPI.begin();
  delay(100);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

  for (uint8_t i = 1; i <= 5; i++)
  {
    Serial.printf("[LoRa] Init %d/5...\r\n", i);
    Serial.flush();
    if (LoRa.begin(LORA_FREQUENCY))
    {
      LoRa.setSpreadingFactor(LORA_SF);
      LoRa.setSignalBandwidth(LORA_BW);
      LoRa.setCodingRate4(LORA_CR);
      LoRa.setTxPower(LORA_TX_POWER);
      LoRa.setSyncWord(LORA_SYNC_WORD);
      Serial.printf("[LoRa] Ready @ %.0fMHz  SF%d  BW%.0fkHz\r\n",
                    LORA_FREQUENCY / 1e6, LORA_SF, LORA_BW / 1e3);
      Serial.flush();
      return true;
    }
    delay(500);
  }
  Serial.println("[LoRa] INIT FAILED — check wiring!\r\n"
                 "       NSS=5 MOSI=23 MISO=19 SCK=18 RST=14 DIO0=26");
  Serial.flush();
  return false;
}

// ════════════════════════════════════════════════════════════
//  LoRa Send — JSON ที่มีข้อมูลทั้ง 2 sensor
// ════════════════════════════════════════════════════════════
static bool lora_send(const SensorReading &s1, const SensorReading &s2)
{
  StaticJsonDocument<512> doc;
  doc["id"] = DEVICE_ID;
  doc["ts"] = millis() / 1000UL;
  doc["pkt"] = ++g_packet_count;
  doc["boot"] = g_boot_count;

  // Sensor 1
  JsonObject j1 = doc.createNestedObject("s1");
  j1["cbar"] = serialized(String(s1.cbar, 2));
  j1["kpa"] = serialized(String(s1.kpa, 2));
  j1["vout"] = serialized(String(s1.vout, 3));
  j1["status"] = soil_status(s1.cbar);
  j1["depth_cm"] = DEPTH_CM_S1;

  // Sensor 2
  JsonObject j2 = doc.createNestedObject("s2");
  j2["cbar"] = serialized(String(s2.cbar, 2));
  j2["kpa"] = serialized(String(s2.kpa, 2));
  j2["vout"] = serialized(String(s2.vout, 3));
  j2["status"] = soil_status(s2.cbar);
  j2["depth_cm"] = DEPTH_CM_S2;

  char buf[512];
  serializeJson(doc, buf);

  LoRa.beginPacket();
  LoRa.print(buf);
  bool ok = LoRa.endPacket();

  Serial.printf("[LoRa] %s → %s\r\n", ok ? "TX OK" : "TX FAIL", buf);
  Serial.flush();
  return ok;
}
#endif // ENABLE_LORA

// ════════════════════════════════════════════════════════════
//  Enter Deep Sleep
// ════════════════════════════════════════════════════════════
static void go_to_sleep()
{
  Serial.printf("[SLP] ไปนอน %llu วินาที... (boot #%lu)\r\n",
                SLEEP_DURATION_S, g_boot_count);
  Serial.flush();
  delay(100); // ให้ Serial flush ก่อน

#if ENABLE_LORA
  LoRa.sleep(); // ลด current draw ของ LoRa module
  delay(10);
#endif

  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_S * 1000000ULL); // microseconds
  esp_deep_sleep_start();
  // ← code ไม่ถึงจุดนี้ — ESP32 reset เมื่อตื่น
}

// ════════════════════════════════════════════════════════════
//  Setup  (= main program เพราะใช้ deep sleep)
// ════════════════════════════════════════════════════════════
void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.flush();

  g_boot_count++;

  // บอกเหตุผลการตื่น
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_TIMER)
    Serial.printf("\r\n=== ตื่นจาก Deep Sleep (boot #%lu) ===\r\n", g_boot_count);
  else
    Serial.printf("\r\n=== Boot ครั้งแรก / Power-on (boot #%lu) ===\r\n", g_boot_count);
  Serial.flush();

  // ── ADC Config ────────────────────────────────────────────
  analogSetAttenuation(ADC_ATTEN);
  analogSetPinAttenuation(ADC_PIN_S1, ADC_ATTEN);
  analogSetPinAttenuation(ADC_PIN_S2, ADC_ATTEN);
  delay(50); // settle

  // ── อ่าน Sensor ทั้งสอง ───────────────────────────────────
  Serial.println("[SENS] กำลังอ่านค่า...");
  Serial.flush();

  SensorReading r1 = read_sensor(ADC_PIN_S1, ZERO_OFFSET_S1, &g_ema_s1);
  SensorReading r2 = read_sensor(ADC_PIN_S2, ZERO_OFFSET_S2, &g_ema_s2);

  // ── Serial Debug ───────────────────────────────────────────
  Serial.printf("[S1 @%dcm] Vout=%.3fV  Vacuum=%.2fkPa  Raw=%.2fcbar  EMA=%.2fcbar  [%s]\r\n",
                DEPTH_CM_S1, r1.vout, r1.kpa, r1.tension, r1.cbar, soil_status(r1.cbar));
  Serial.printf("[S2 @%dcm] Vout=%.3fV  Vacuum=%.2fkPa  Raw=%.2fcbar  EMA=%.2fcbar  [%s]\r\n",
                DEPTH_CM_S2, r2.vout, r2.kpa, r2.tension, r2.cbar, soil_status(r2.cbar));
  Serial.flush();

  // ── LoRa Transmit ──────────────────────────────────────────
#if ENABLE_LORA
  if (lora_init())
  {
    delay(500);
    lora_send(r1, r2);
    delay(500);
  }
  else
  {
    Serial.println("[ERR] LoRa failed — skip TX, going to sleep anyway");
    Serial.flush();
  }
#else
  Serial.println("[LoRa] Disabled — Serial only");
  Serial.flush();
#endif

  // ── Deep Sleep ─────────────────────────────────────────────
  go_to_sleep();
}

void loop() {}