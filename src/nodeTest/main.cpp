#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// พินตามที่คุณระบุไว้
#define SCK 18
#define MISO 19
#define MOSI 23
#define SS 5
#define RST 14
#define DIO0 26

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    delay(1000);
    Serial.println("\n--- LoRa Hardware Diagnostic ---");

    // 1. บังคับ Reset ชิป LoRa (Manual Reset)
    // หลายครั้งที่ชิปค้างเพราะไฟกระชากตอนเริ่ม
    pinMode(RST, OUTPUT);
    digitalWrite(RST, LOW);
    delay(100);
    digitalWrite(RST, HIGH);
    delay(100);
    Serial.println("1. Manual Reset: DONE");

    // 2. เริ่ม SPI และกำหนดขาให้ชัดเจน
    SPI.begin(SCK, MISO, MOSI, SS);
    LoRa.setPins(SS, RST, DIO0);
    Serial.println("2. SPI Bus: INITIALIZED");

    // 3. ตรวจสอบ Register 0x42 (Version Register)
    // ชิป SX127x ทุกตัวต้องตอบค่า 0x12 (18) กลับมา
    Serial.print("3. Checking LoRa Chip Version...");
    if (!LoRa.begin(433E6))
    {
        Serial.println(" [FAIL]");

        // วิเคราะห์สาเหตุ
        Serial.println("\n--- DIAGNOSIS REPORT ---");
        Serial.println("- หาก FAIL ทันที: สาย SS (5) หรือ SCK (18) อาจจะหลวม");
        Serial.println("- หากค้างที่จุดนี้: สาย MISO (19) หรือ MOSI (23) อาจจะสลับกัน");
        Serial.println("- ตรวจสอบไฟเลี้ยง: วัดแรงดันที่ขา VCC ของ LoRa ต้องได้ 3.3V เป๊ะๆ");

        while (1)
            delay(1000);
    }

    Serial.println(" [SUCCESS]");
    Serial.println("4. LoRa Status: READY AND LISTENING");

    LoRa.setSyncWord(0xF3);
    LoRa.receive();
}

void loop()
{
    int packetSize = LoRa.parsePacket();
    if (packetSize)
    {
        Serial.print("Received packet! RSSI: ");
        Serial.println(LoRa.packetRssi());
    }
}