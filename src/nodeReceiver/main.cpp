#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ================= WiFi =================
const char *ssid = "@JumboPlusIoT";
const char *password = "tensiometer";

// ================= Google Script =================
const char *SCRIPT_URL = "https://script.google.com/macros/s/AKfycbzTYjY1S5XtG9U1DtGI-ya6R1pYr1D0v0JqZn3wluoiE5D1c9sxSLqJOgVUx2t2hhYVHg/exec";

// ================= LoRa =================
#define SS 5
#define RST 14
#define DIO0 26

// ================= WiFi Connect =================
bool ensureWiFi()
{
  if (WiFi.status() == WL_CONNECTED)
    return true;

  WiFi.disconnect();
  WiFi.begin(ssid, password);

  Serial.print("[WiFi] Connecting");
  unsigned long t = millis();

  while (WiFi.status() != WL_CONNECTED)
  {
    if (millis() - t > 10000)
    {
      Serial.println(" FAIL");
      return false;
    }
    delay(500);
    Serial.print(".");
  }

  Serial.println(" OK");
  return true;
}

// ================= SEND TO SHEET =================
void sendToSheet(JsonDocument &doc, int rssiVal)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.begin(ssid, password);
    delay(2000);
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;

  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(15000);

  String id = doc["id"] | "unknown";
  int pkt = doc["pkt"] | 0;

  JsonObject s1 = doc["s1"];
  JsonObject s2 = doc["s2"];

  String url = String(SCRIPT_URL) +
               "?id=" + id +
               "&pkt=" + String(pkt) +
               "&rssi=" + String(rssiVal) +

               "&s1_cbar=" + String((float)s1["cbar"], 2) +
               "&s1_kpa=" + String((float)s1["kpa"], 2) +
               "&s1_status=" + String((const char *)s1["status"]) +

               "&s2_cbar=" + String((float)s2["cbar"], 2) +
               "&s2_kpa=" + String((float)s2["kpa"], 2) +
               "&s2_status=" + String((const char *)s2["status"]);

  Serial.println("SEND:");
  Serial.println(url);

  http.begin(client, url);
  http.addHeader("Connection", "close");

  int code = http.GET();

  Serial.print("HTTP: ");
  Serial.println(code);

  String payload = http.getString();
  Serial.println(payload);

  http.end();
}

// ================= SETUP =================
void setup()
{
  Serial.begin(115200);

  WiFi.begin(ssid, password);

  SPI.begin();
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(433E6))
  {
    Serial.println("LoRa fail");
    while (1)
      ;
  }

  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setSyncWord(0xF3);

  Serial.println("Ready");
}

// ================= LOOP =================
void loop()
{
  int size = LoRa.parsePacket();

  if (size)
  {
    int currentRssi = LoRa.packetRssi();

    String raw = "";

    while (LoRa.available())
      raw += (char)LoRa.read();

    Serial.println("RX:");
    Serial.println(raw);

    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, raw))
    {
      Serial.println("JSON ERROR");
      return;
    }

    sendToSheet(doc, currentRssi);
  }

  delay(10);
}