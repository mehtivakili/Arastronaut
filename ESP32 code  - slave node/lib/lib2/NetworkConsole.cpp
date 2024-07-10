#include "NetworkConsole.h"
#include <stdarg.h> // Include for variable arguments handling

NetworkConsole::NetworkConsole(int port)
    : _port(port), _server(new WiFiServer(port)) {}

void NetworkConsole::begin() {
    // Start the TCP server
    _server->begin();
    _server->setNoDelay(true);

    // Setup OTA
    setupOTA();
}

void NetworkConsole::handleClient() {
    if (_server->hasClient()) {
        if (!_client || !_client.connected()) {
            if (_client) {
                _client.stop();
            }
            _client = _server->available();
            if (_client) {
                _client.println("Welcome to the ESP32 Network Console!");
                _client.flush();
            }
        }
    }
}

void NetworkConsole::print(const char* message) {
    handleClient();
    if (_client && _client.connected()) {
        _client.print(message);
        _client.flush();
    }
}

void NetworkConsole::println(const char* message) {
    handleClient();
    if (_client && _client.connected()) {
        _client.println(message);
        _client.flush();
    }
}

void NetworkConsole::print(String message) {
    print(message.c_str());
}

void NetworkConsole::println(String message) {
    println(message.c_str());
}

void NetworkConsole::printf(const char* format, ...) {
    handleClient();
    if (_client && _client.connected()) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        // Replace '\n' with '\r\n' to ensure proper line breaks in PuTTY
        String output(buffer);
        output.replace("\n", "\r\n");

        _client.print(output);
        _client.flush();
    }
}

void NetworkConsole::setupOTA() {
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {
            type = "filesystem";
        }
        Serial.println("Start updating " + type);
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\nEnd");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            Serial.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
            Serial.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
            Serial.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
            Serial.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
            Serial.println("End Failed");
        }
    });

    ArduinoOTA.begin();
}
