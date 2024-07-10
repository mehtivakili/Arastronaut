#include "web_interface.h"
#include "config.h"

WebServer server(80); // Define the server object

void setupWebServer() {
    server.on("/", handleRoot);
    server.on("/config", handleConfig);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("HTTP server started");
}

void handleRoot() {
    String html = "<html><head>"
                  "<style>"
                  "body { display: flex; }"
                  "#config { width: 50%; }"
                  "#console { width: 50%; background-color: #f4f4f4; padding: 10px; }"
                  "</style>"
                  "</head><body>"
                  "<div id=\"config\">"
                  "<h1>Configuration</h1>"
                  "<form action=\"/config\" method=\"post\">";

    // UWB Pre-calibration
    html += "<label for=\"UWB_PRECALIBRATION\">UWB Pre-calibration:</label>"
            "<input type=\"checkbox\" id=\"UWB_PRECALIBRATION\" name=\"UWB_PRECALIBRATION\"";
    if (UWB_PRECALIBRATION) html += " checked";
    html += "><br><br>";

    // Device Type
    html += "<label for=\"DEVICE_TYPE\">Device Type (1 for TAG, 0 for ANCHOR):</label>"
            "<input type=\"number\" id=\"DEVICE_TYPE\" name=\"DEVICE_TYPE\" value=\"" + String(DEVICE_TYPE) + "\"><br><br>";

    // Update Interval
    html += "<label for=\"updateInterval\">Update Interval:</label>"
            "<input type=\"number\" id=\"updateInterval\" name=\"updateInterval\" value=\"" + String(updateInterval) + "\"><br><br>";

    // IMU Calibration
    html += "<label for=\"IMU_CALIBRATED\">IMU Calibrated:</label>"
            "<input type=\"checkbox\" id=\"IMU_CALIBRATED\" name=\"IMU_CALIBRATED\"";
    if (IMU_CALIBRATED) html += " checked";
    html += "><br><br>";

    // Actual Distance for UWB
    html += "<label for=\"actual_distance\">Actual Distance:</label>"
            "<input type=\"number\" id=\"actual_distance\" name=\"actual_distance\" step=\"0.01\" value=\"" + String(actual_distance) + "\"><br><br>";

    // IMU Calibration Parameters
    html += "<h2>IMU Calibration Parameters</h2>";

    // Ka
    html += "<h3>Ka</h3>";
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            String id = "Ka" + String(i) + String(j);
            html += "<label for=\"" + id + "\">Ka[" + String(i) + "][" + String(j) + "]:</label>"
                    "<input type=\"number\" id=\"" + id + "\" name=\"" + id + "\" step=\"0.00001\" value=\"" + String(Ka[i][j]) + "\"><br>";
        }
    }

    // Ba
    html += "<h3>Ba</h3>";
    for (int i = 0; i < 3; i++) {
        String id = "Ba" + String(i);
        html += "<label for=\"" + id + "\">Ba[" + String(i) + "]:</label>"
                "<input type=\"number\" id=\"" + id + "\" name=\"" + id + "\" step=\"0.00001\" value=\"" + String(acce_bias[i]) + "\"><br>";
    }

    // Tg
    html += "<h3>Tg</h3>";
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            String id = "Tg" + String(i) + String(j);
            html += "<label for=\"" + id + "\">Tg[" + String(i) + "][" + String(j) + "]:</label>"
                    "<input type=\"number\" id=\"" + id + "\" name=\"" + id + "\" step=\"0.00001\" value=\"" + String(Tg[i][j]) + "\"><br>";
        }
    }

    // Kg
    html += "<h3>Kg</h3>";
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            String id = "Kg" + String(i) + String(j);
            html += "<label for=\"" + id + "\">Kg[" + String(i) + "][" + String(j) + "]:</label>"
                    "<input type=\"number\" id=\"" + id + "\" name=\"" + id + "\" step=\"0.00001\" value=\"" + String(Kg[i][j]) + "\"><br>";
        }
    }

    // Bb
    html += "<h3>Bb</h3>";
    for (int i = 0; i < 3; i++) {
        String id = "Bb" + String(i);
        html += "<label for=\"" + id + "\">Bb[" + String(i) + "]:</label>"
                "<input type=\"number\" id=\"" + id + "\" name=\"" + id + "\" step=\"0.00001\" value=\"" + String(gyro_bias[i]) + "\"><br>";
    }

    html += "<input type=\"submit\" value=\"Set Configuration\">"
            "</form></div>";

    // Console
    html += "<div id=\"console\">"
            "<h2>Console</h2>"
            "<pre id=\"console-output\"></pre>"
            "</div>"
            "<script>"
            "function refreshConsole() {"
            "  fetch('/console').then(response => response.text()).then(data => {"
            "    document.getElementById('console-output').innerText = data;"
            "  });"
            "}"
            "setInterval(refreshConsole, 1000);"
            "</script>"
            "</body></html>";

    server.send(200, "text/html", html);
}

void handleConfig() {
    if (server.hasArg("UWB_PRECALIBRATION")) {
        UWB_PRECALIBRATION = server.arg("UWB_PRECALIBRATION") == "on";
    }
    if (server.hasArg("DEVICE_TYPE")) {
        DEVICE_TYPE = server.arg("DEVICE_TYPE").toInt();
    }
    if (server.hasArg("updateInterval")) {
        updateInterval = server.arg("updateInterval").toInt();
    }
    if (server.hasArg("IMU_CALIBRATED")) {
        IMU_CALIBRATED = server.arg("IMU_CALIBRATED") == "on";
    }
    if (server.hasArg("actual_distance")) {
        actual_distance = server.arg("actual_distance").toFloat();
    }

    // Update IMU Calibration Parameters
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            String id = "Ka" + String(i) + String(j);
            if (server.hasArg(id)) {
                Ka[i][j] = server.arg(id).toFloat();
            }
        }
    }
    for (int i = 0; i < 3; i++) {
        String id = "Ba" + String(i);
        if (server.hasArg(id)) {
            acce_bias[i] = server.arg(id).toFloat();
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            String id = "Tg" + String(i) + String(j);
            if (server.hasArg(id)) {
                Tg[i][j] = server.arg(id).toFloat();
            }
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            String id = "Kg" + String(i) + String(j);
            if (server.hasArg(id)) {
                Kg[i][j] = server.arg(id).toFloat();
            }
        }
    }
    for (int i = 0; i < 3; i++) {
        String id = "Bb" + String(i);
        if (server.hasArg(id)) {
            gyro_bias[i] = server.arg(id).toFloat();
        }
    }

    String response = "<html><body>"
                      "<h1>Configuration Updated</h1>"
                      "<a href=\"/\">Back to Configuration</a>"
                      "</body></html>";

    server.send(200, "text/html", response);
}

void handleNotFound() {
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";
    for (uint8_t i = 0; i < server.args(); i++) {
        message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }
    server.send(404, "text/plain", message);
}
