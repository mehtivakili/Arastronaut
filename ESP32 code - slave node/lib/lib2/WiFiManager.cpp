#include "WiFiManager.h"
const char* ap_ssid = "ESP32_Config";
const char* ap_password = "12345678";
Preferences preferences;
// HTML page for configuration
const char* configPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Network Config</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }
    form { display: inline-block; margin-top: 20px; }
    label, input { display: block; margin: 10px 0; }
    ul { text-align: left; display: inline-block; }
    #error { color: red; }
  </style>
</head>
<body>
  <h1>ESP32 Network Configuration</h1>
  <form id="wifiForm">
    <label for="ssid">SSID:</label>
    <input type="text" id="ssid" name="ssid" required>
    <label for="password">Password:</label>
    <input type="password" id="password" name="password" required>
    <button type="submit">Save</button>
  </form>
  <p id="error"></p>
  <h2>Available Networks</h2>
  <div id="networks">Scanning...</div>
  <script>
    async function fetchNetworks() {
      try {
        const response = await fetch('/networks');
        const data = await response.json();
        console.log('Fetched networks:', data); // Debug
        const networksList = data.networks.map(network => 
          `<li>${network.ssid} (${network.rssi}) ${network.encryption === 'open' ? '' : '*'}</li>`
        ).join('');
        document.getElementById('networks').innerHTML = '<ul>' + networksList + '</ul>';
      } catch (error) {
        console.error('Error fetching networks:', error);
      }
    }

    setInterval(fetchNetworks, 5000); // Fetch networks every 5 seconds

    document.getElementById('wifiForm').addEventListener('submit', async function(e) {
      e.preventDefault();
      const ssid = document.getElementById('ssid').value;
      const password = document.getElementById('password').value;
      console.log(`SSID: ${ssid}, Password: ${password}`); // Debug
      const response = await fetch('/setWiFi', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: `ssid=${ssid}&password=${password}`
      });
      const result = await response.text();
      console.log('Set WiFi result:', result); // Debug
      if (result.includes('Failed to connect')) {
        document.getElementById('error').innerText = 'Password is incorrect. Retrying in 6 seconds...';
        let countdown = 6;
        const interval = setInterval(() => {
          countdown--;
          if (countdown > 0) {
            document.getElementById('error').innerText = `Password is incorrect. Retrying in ${countdown} seconds...`;
          } else {
            document.getElementById('error').innerText = '';
            clearInterval(interval);
          }
        }, 1000);
        document.getElementById('password').value = '';
        fetchNetworks(); // Ensure network list is refreshed
      } else {
        document.getElementById('error').innerText = '';
        alert(result);
      }
    });

    // Initial fetch
    fetchNetworks();
  </script>
</body>
</html>
)rawliteral";

// HTML page for welcome message
const char* welcomePage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Welcome to ESP32</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }
    .welcome { font-size: 24px; color: #333; }
    button { padding: 10px 20px; margin-top: 20px; }
  </style>
</head>
<body>
  <h1 class="welcome">Welcome to ESP32</h1>
  <p>Your ESP32 is successfully connected to the network.</p>
  <form action="/forgetWiFi" method="POST">
    <button type="submit">Forget Network</button>
  </form>
  <button onclick="window.location.href='/plot'">Go to Plot Page</button>
</body>
</html>
)rawliteral";


WiFiManager::WiFiManager() : server(80) {}

void WiFiManager::begin() {
  Serial.begin(115200);

  if (!connectToSavedWiFi()) {
    setupAccessPoint();
  } else {
    if (MDNS.begin("esp32")) {
      Serial.println("mDNS responder started");
      Serial.println("You can access the device at http://esp32.local");
    } else {
      Serial.println("Error setting up mDNS responder!");
    }
    server.on("/", [this]() {
      server.send(200, "text/html", welcomePage);
    });
    server.on("/forgetWiFi", HTTP_POST, [this]() { handleForgetWiFi(); });
    server.begin();
    Serial.println("HTTP server started");
  }
}

void WiFiManager::handleClient() {
  server.handleClient();
}

void WiFiManager::setupAccessPoint() {
  WiFi.softAP(ap_ssid, ap_password);
  Serial.println("Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  setupWebServer();
}

void WiFiManager::setupWebServer() {
  server.on("/", [this]() {
    server.send(200, "text/html", configPage);
  });
  server.on("/setWiFi", HTTP_POST, [this]() { handleSetWiFi(); });
  server.on("/forgetWiFi", HTTP_POST, [this]() { handleForgetWiFi(); });
  server.on("/networks", [this]() { handleScanNetworks(); });
  server.begin();
  Serial.println("HTTP server started");
}

void WiFiManager::handleScanNetworks() {
  int n = WiFi.scanNetworks(false, true);
  StaticJsonDocument<2048> doc;
  JsonArray networks = doc.createNestedArray("networks");

  Serial.println("Scanned networks:");
  for (int i = 0; i < n; ++i) {
    JsonObject network = networks.createNestedObject();
    network["ssid"] = WiFi.SSID(i);
    network["rssi"] = WiFi.RSSI(i);
    network["encryption"] = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "open" : "encrypted";

    Serial.print("SSID: ");
    Serial.print(WiFi.SSID(i));
    Serial.print(", RSSI: ");
    Serial.print(WiFi.RSSI(i));
    Serial.print(", Encryption: ");
    Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "open" : "encrypted");
  }

  String jsonResponse;
  serializeJson(doc, jsonResponse);
  server.send(200, "application/json", jsonResponse);
  Serial.println("Sent JSON response");
}

void WiFiManager::handleSetWiFi() {
  if (server.method() == HTTP_POST) {
    String ssid = server.arg("ssid");
    String password = server.arg("password");

    Serial.print("Received SSID: ");
    Serial.println(ssid);
    Serial.print("Received Password: ");
    Serial.println(password);

    // Set custom hostname
    WiFi.setHostname("Arastronaut");
    
    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.print("Connecting to ");
    Serial.println(ssid);
    for (int i = 0; i < 10; i++) {
      if (WiFi.status() == WL_CONNECTED) {
            // Set custom hostname
             WiFi.setHostname("Arastronaut");  
        Serial.println("Connected to WiFi");

        // Save credentials to preferences
        preferences.begin("WiFiCreds", false);
        preferences.putString("ssid", ssid);
        preferences.putString("password", password);
        preferences.end();

        // Respond to client and reboot
        server.send(200, "text/html", "Wi-Fi credentials saved. Rebooting...");
        delay(2000);
        ESP.restart();
        return;
      }
      delay(500);
      Serial.print(".");
    }
    Serial.println("Failed to connect to WiFi");
    server.send(200, "text/html", "Failed to connect to WiFi. Please check your credentials.");
    handleScanNetworks();

    // Restart the Wi-Fi setup if connection fails
    WiFi.disconnect();
    WiFi.softAP(ap_ssid, ap_password);
    Serial.println("Restarting Access Point Mode");
    server.send(200, "text/html", configPage);
  }
}

void WiFiManager::handleForgetWiFi() {
  // Clear credentials from preferences
  preferences.begin("WiFiCreds", false);
  preferences.clear();
  preferences.end();

  // Respond to client and reboot
  String response = "Wi-Fi credentials forgotten. Rebooting...";
  server.send(200, "text/html", response);

  delay(2000);
  ESP.restart();
}

bool WiFiManager::connectToSavedWiFi() {
  preferences.begin("WiFiCreds", true);
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("password", "");
  preferences.end();

  if (ssid.length() > 0) {
    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.print("Connecting to ");
    Serial.println(ssid);
    for (int i = 0; i < 10; i++) {
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Connected to WiFi");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        return true;
      }
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("Failed to connect to WiFi");
  }
  return false;
}

// Add this method to provide access to the server instance
WebServer& WiFiManager::getServer() {
  return server;
}
