#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

void sendToSheet(String id, int ts, int pkt, float cbar, float kpa, float vout, String status);

// ================= WiFi =================
const char *ssid = "@JumboPlusIoT";
const char *password = "tensiometer";

// ================= Google Script =================
const char *SCRIPT_URL = "https://script.google.com/macros/s/AKfycbzfP2i5CG9oVMwXx1hXi_opYPGzq0RWdhc-TvGWAeMijKWAuSNJytxU-CSdVqTmjMH7Mw/exec";

// ================= LoRa =================
#define SS 5
#define RST 14
#define DIO0 26

// ================= SETUP =================
void setup()
{
  Serial.begin(115200);

  // ----- WiFi -----
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting WiFi");
  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000)
  {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWiFi Connected");
  }
  else
  {
    Serial.println("\nWiFi FAILED");
  }

  // ----- LoRa -----
  SPI.begin();
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(433E6))
  {
    Serial.println("❌ LoRa init failed");
    while (1)
      ;
  }

  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setTxPower(17);
  LoRa.setSyncWord(0xF3); // 🔥 สำคัญ

  Serial.println("✅ Receiver Ready");
}

// ================= LOOP =================
void loop()
{
  int packetSize = LoRa.parsePacket();

  if (packetSize)
  {
    String received = "";

    while (LoRa.available())
    {
      received += (char)LoRa.read();
    }

    Serial.println("📡 Received:");
    Serial.println(received);

    // -------- Parse JSON --------
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, received);

    if (err)
    {
      Serial.println("❌ JSON Parse Failed");
      return;
    }

    // -------- Extract values --------
    String id = doc["id"];
    int ts = doc["ts"];
    int pkt = doc["pkt"];
    float cbar = doc["cbar"].as<float>();
    float kpa = doc["kpa"].as<float>();
    float vout = doc["vout"].as<float>();
    String status = doc["status"];

    Serial.println("---- Parsed ----");
    Serial.println(id);
    Serial.println(cbar);
    Serial.println(status);

    // -------- Send to Google Sheet --------
    sendToSheet(id, ts, pkt, cbar, kpa, vout, status);
  }

  delay(10);
}

// ================= HTTP POST =================
void sendToSheet(String id, int ts, int pkt, float cbar, float kpa, float vout, String status)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi lost, reconnecting...");
    WiFi.reconnect();
    delay(2000);
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  String url = String(SCRIPT_URL) +
               "?id=" + id +
               "&ts=" + String(ts) +
               "&pkt=" + String(pkt) +
               "&cbar=" + String(cbar, 2) +
               "&kpa=" + String(kpa, 2) +
               "&vout=" + String(vout, 3) +
               "&status=" + status;

  Serial.println("📤 GET:");
  Serial.println(url);

  http.begin(client, url);

  int httpCode = http.GET();

  Serial.print("HTTP Response: ");
  Serial.println(httpCode);

  String payload = http.getString();
  Serial.println(payload);

  http.end();
}