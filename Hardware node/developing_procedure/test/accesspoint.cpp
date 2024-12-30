#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// Replace with your network credentials
const char* ssid = "ESP32-Access-Point";
const char* password = "12345678";

WebServer server(80);

String networkList = "";

// Function to scan for available networks
void scanNetworks() {
  int n = WiFi.scanNetworks();
  networkList = "";
  for (int i = 0; i < n; ++i) {
    networkList += "<option value=\"" + WiFi.SSID(i) + "\">" + WiFi.SSID(i) + "</option>";
  }
}

// HTML content for the main page
const char* mainPage = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <title>ESP32 Web Setup</title>
    <style>
      body { font-family: Arial, Helvetica, sans-serif; text-align: center; }
      .error { color: red; }
    </style>
    <script>
      function refreshNetworks() {
        fetch('/scan').then(response => response.text()).then(data => {
          document.getElementById('networkList').innerHTML = data;
        });
      }
    </script>
  </head>
  <body>
    <h1>ESP32 WiFi Setup</h1>
    <form action="/connect" method="post">
      <label for="ssid">SSID:</label>
      <select id="networkList" name="ssid">
        %NETWORKS%
      </select>
      <br><br>
      <label for="password">Password:</label>
      <input type="password" id="password" name="password">
      <br><br>
      <input type="submit" value="Connect">
    </form>
    <br>
    <button onclick="refreshNetworks()">Refresh Networks</button>
    <br><br>
    %MESSAGE%
  </body>
</html>
)rawliteral";

// Function to handle the main page
void handleRoot() {
  String page = mainPage;
  page.replace("%NETWORKS%", networkList);
  page.replace("%MESSAGE%", "");
  server.send(200, "text/html", page);
}

// Function to handle the connection request
void handleConnect() {
  String ssid = server.arg("ssid");
  String pass = server.arg("password");

  WiFi.begin(ssid.c_str(), pass.c_str());

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    delay(1000);
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    server.sendHeader("Location", "/success");
    server.send(303);
  } else {
    String page = mainPage;
    page.replace("%NETWORKS%", networkList);
    page.replace("%MESSAGE%", "<p class=\"error\">Wrong SSID or Password!</p>");
    server.send(200, "text/html", page);
  }
}

// Function to handle the success page
void handleSuccess() {
  String page = "<html><body><h1>Welcome to local server ESP32</h1></body></html>";
  server.send(200, "text/html", page);
}

// Function to handle the network scan request
void handleScan() {
  scanNetworks();
  server.send(200, "text/plain", networkList);
}

void setup() {
  Serial.begin(115200);
  
  // Setting up the access point
  WiFi.softAP(ssid, password);

  // Start the server
  server.on("/", handleRoot);
  server.on("/connect", HTTP_POST, handleConnect);
  server.on("/success", handleSuccess);
  server.on("/scan", handleScan);

  server.begin();

  // Scan networks at startup
  scanNetworks();

  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
