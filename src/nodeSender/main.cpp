// Sender node code for LoRa communication example
/*
  #include <Arduino.h>
  #include <SPI.h>
  #include <LoRa.h>

  #define SS 5
  #define RST 14
  #define DIO0 26

  void setup()
  {
    Serial.begin(115200);

    LoRa.setPins(SS, RST, DIO0);

    if (!LoRa.begin(433E6))
    {
      Serial.println("LoRa init failed");
      while (1)
        ;
    }

    LoRa.setTxPower(20);
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(125E3);

    Serial.println("NodeSender Ready");
  }

  void loop()
  {

    Serial.println("Send: Hello Node2");

    LoRa.beginPacket();
    LoRa.print("Hello Node2");
    LoRa.endPacket();

    delay(1000);

    int packetSize = LoRa.parsePacket();
    if (packetSize)
    {

      Serial.print("Reply: ");

      while (LoRa.available())
      {
        Serial.print((char)LoRa.read());
      }

      Serial.println();
    }

    delay(2000);
  }
*/

/**
 * LoRa Hardware Test — Ra-02 (SX1278)
 * ─────────────────────────────────────
 * ทดสอบ SPI + SX1278 โดยตรง ไม่ใช้ LoRa library
 *
 * ผลที่ควรได้:
 *   RegVersion (0x42) = 0x12  → SX1278 ตอบสนอง, SPI ทำงาน ✅
 *   RegVersion (0x42) = 0x00  → MISO ลอย หรือ NSS/SCK/MOSI ผิดขา ❌
 *   RegVersion (0x42) = 0xFF  → MISO ค้างที่ HIGH (short to 3.3V) ❌
 *
 * Wiring (Ra-02):
 *   Ra-02 NSS  → GPIO5
 *   Ra-02 MOSI → GPIO23
 *   Ra-02 MISO → GPIO19
 *   Ra-02 SCK  → GPIO18
 *   Ra-02 RST  → GPIO14
 *   Ra-02 3.3V → 3.3V  (ห้ามต่อ 5V)
 *   Ra-02 GND  → GND
 */

#include <Arduino.h>
#include <SPI.h>

// ───────── PIN CONFIG ─────────
#define PIN_NSS 5
#define PIN_RST 14
#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_SCK 18

// ───────── REGISTERS ─────────
#define REG_VERSION 0x42
#define REG_OP_MODE 0x01

// ───────── SPI READ ─────────
uint8_t spi_read(uint8_t reg)
{
  digitalWrite(PIN_NSS, LOW);
  delayMicroseconds(5);

  SPI.transfer(reg & 0x7F); // MSB=0 → read
  uint8_t val = SPI.transfer(0x00);

  digitalWrite(PIN_NSS, HIGH);
  delayMicroseconds(5);

  return val;
}

// ───────── SPI WRITE ─────────
void spi_write(uint8_t reg, uint8_t val)
{
  digitalWrite(PIN_NSS, LOW);
  delayMicroseconds(5);

  SPI.transfer(reg | 0x80); // MSB=1 → write
  SPI.transfer(val);

  digitalWrite(PIN_NSS, HIGH);
  delayMicroseconds(5);
}

// ───────── RESET ─────────
void lora_reset()
{
  digitalWrite(PIN_RST, LOW);
  delay(50);
  digitalWrite(PIN_RST, HIGH);
  delay(50);
}

// ───────── SETUP ─────────
void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== SX1278 (Ra-02) SPI DEBUG ===\n");

  // ── GPIO ──
  pinMode(PIN_NSS, OUTPUT);
  digitalWrite(PIN_NSS, HIGH);

  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, HIGH);

  // ใช้ internal pull-up ช่วย stabilize (optional)
  pinMode(PIN_MISO, INPUT_PULLUP);

  // ── SPI INIT ──
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  SPI.setFrequency(1000000); // 1 MHz (debug mode)
  SPI.setDataMode(SPI_MODE0);

  Serial.println("[SPI] OK");

  // ── RESET ──
  Serial.println("[RST] Resetting...");
  lora_reset();
  Serial.println("[RST] Done");

  // ── READ VERSION ──
  uint8_t ver = spi_read(REG_VERSION);

  Serial.printf("\n[REG] Version = 0x%02X\n", ver);

  if (ver == 0x12)
  {
    Serial.println("✅ SX1278 detected (SPI OK)\n");
  }
  else if (ver == 0x00)
  {
    Serial.println("❌ ERROR: 0x00 (MISO not responding)");
    Serial.println("→ Check wiring / power / NSS\n");
  }
  else if (ver == 0xFF)
  {
    Serial.println("❌ ERROR: 0xFF (MISO shorted to VCC)\n");
  }
  else
  {
    Serial.printf("⚠️ Unexpected value: 0x%02X\n\n", ver);
  }

  // ── TEST WRITE/READ ──
  Serial.println("[TEST] Write + Read RegOpMode");

  spi_write(REG_OP_MODE, 0x80); // LoRa sleep mode
  delay(10);

  uint8_t op = spi_read(REG_OP_MODE);
  Serial.printf("RegOpMode = 0x%02X\n\n", op);

  // ── DUMP REGISTERS ──
  Serial.println("[DUMP] First 10 registers:");
  for (uint8_t i = 0; i <= 0x09; i++)
  {
    Serial.printf("0x%02X : 0x%02X\n", i, spi_read(i));
    delay(5);
  }

  Serial.println("\n=== Setup Done ===\n");
}

// ───────── LOOP ─────────
void loop()
{
  delay(3000);

  uint8_t ver = spi_read(REG_VERSION);

  Serial.printf("[POLL] Version = 0x%02X ", ver);

  if (ver == 0x12)
    Serial.println("✅");
  else
    Serial.println("❌ connection lost");
}