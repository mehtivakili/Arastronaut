#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include <esp_netif.h>

const char* ssid = "ESP32_AP";
const char* password = "123456789";

WebServer server(80);

void handleRoot() {
    String html = "<html><body><h1>Connected Clients</h1><ul>";

    wifi_sta_list_t wifi_sta_list;
    esp_netif_sta_list_t adapter_sta_list;

    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_netif_get_sta_list(&wifi_sta_list, &adapter_sta_list);

    for (int i = 0; i < adapter_sta_list.num; i++) {
        esp_netif_sta_info_t station = adapter_sta_list.sta[i];
        char macStr[18];
        sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", MAC2STR(station.mac));
        html += "<li>MAC: " + String(macStr) + " - IP: " + IPAddress(station.ip.addr).toString() + "</li>";
    }

    html += "</ul>";

    html += "<h2>Modify Client Settings</h2>";
    html += "<form action=\"/modify\" method=\"POST\">";
    html += "IP Address: <input type=\"text\" name=\"ip\"><br>";
    html += "Setting: <input type=\"text\" name=\"setting\"><br>";
    html += "<input type=\"submit\" value=\"Apply\">";
    html += "</form></body></html>";

    server.send(200, "text/html", html);
}

void handleModify() {
    if (server.hasArg("ip") && server.hasArg("setting")) {
        String ip = server.arg("ip");
        String setting = server.arg("setting");

        HTTPClient http;
        String url = "http://" + ip + "/modify?setting=" + setting;
        http.begin(url);

        int httpCode = http.GET();
        if (httpCode > 0) {
            String payload = http.getString();
            server.send(200, "text/plain", "Request sent to " + ip + "\nResponse: " + payload);
        } else {
            server.send(500, "text/plain", "Failed to connect to " + ip);
        }
        http.end();
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.softAP(ssid, password);

    server.on("/", handleRoot);
    server.on("/modify", HTTP_POST, handleModify);
    server.begin();

    Serial.println("HTTP server started");
}

void loop() {
    server.handleClient();
}
