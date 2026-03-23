#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ================= WiFi =================
const char *ssid = "@JumboPlusIoT";
const char *password = "tensiometer";

// ================= Google Script =================
const char *SCRIPT_URL = "https://script.google.com/macros/s/AKfycbw9K2NIVjlnWoMehJeh6RBJbwIAwIz-vkG1OUlSYJQQUw5v8b3xtSvQ2jybB2HFzQM9/exec";

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
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");

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
}

// ================= HTTP POST =================
void sendToSheet(String id, int ts, int pkt, float cbar, float kpa, float vout, String status)
{

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi lost, reconnecting...");
    WiFi.reconnect();
    return;
  }

  HTTPClient http;
  http.begin(SCRIPT_URL);
  http.addHeader("Content-Type", "application/json");

  // 🔥 JSON ที่จะส่งไป Google Sheet
  String json = "{";
  json += "\"id\":\"" + id + "\",";
  json += "\"ts\":" + String(ts) + ",";
  json += "\"pkt\":" + String(pkt) + ",";
  json += "\"cbar\":" + String(cbar, 2) + ",";
  json += "\"kpa\":" + String(kpa, 2) + ",";
  json += "\"vout\":" + String(vout, 3) + ",";
  json += "\"status\":\"" + status + "\"";
  json += "}";

  Serial.println("📤 POST:");
  Serial.println(json);

  int httpCode = http.POST(json);

  Serial.print("HTTP Response: ");
  Serial.println(httpCode);

  http.end();
}