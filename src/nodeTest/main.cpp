#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>

#define SS 5
#define RST 14
#define DIO0 26

uint32_t packetCount = 0;

void setup()
{
    Serial.begin(115200);

    SPI.begin();
    LoRa.setPins(SS, RST, DIO0);

    if (!LoRa.begin(433E6))
    {
        Serial.println("❌ LoRa init failed");
        while (1)
            ;
    }

    LoRa.setTxPower(17);
    LoRa.setSpreadingFactor(10);
    LoRa.setSignalBandwidth(125E3);

    Serial.println("✅ Fake Sender Ready");
}

void loop()
{
    // -------- Fake sensor values --------
    float cbar = random(10, 80) + random(0, 100) / 100.0; // 10–80
    float kpa = cbar;
    float vout = random(200, 400) / 100.0; // 2.00 – 4.00 V

    String status;
    if (cbar < 10)
        status = "SATURATED";
    else if (cbar < 30)
        status = "FIELD_CAPACITY";
    else if (cbar < 60)
        status = "OPTIMAL";
    else if (cbar < 80)
        status = "STRESS_SOON";
    else
        status = "IRRIGATE_NOW";

    // -------- Create JSON --------
    StaticJsonDocument<256> doc;
    doc["id"] = "TENS-01";
    doc["ts"] = millis() / 1000;
    doc["pkt"] = ++packetCount;
    doc["cbar"] = String(cbar, 2);
    doc["kpa"] = String(kpa, 2);
    doc["vout"] = String(vout, 3);
    doc["status"] = status;

    char buffer[256];
    serializeJson(doc, buffer);

    // -------- Send via LoRa --------
    Serial.println("📡 Sending:");
    Serial.println(buffer);

    LoRa.beginPacket();
    LoRa.print(buffer);
    LoRa.endPacket();

    delay(5000); // ส่งทุก 5 วิ
}