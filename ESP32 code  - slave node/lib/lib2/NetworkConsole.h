#ifndef NETWORK_CONSOLE_H
#define NETWORK_CONSOLE_H

#include <WiFi.h>
#include <WiFiServer.h>
#include <ArduinoOTA.h>

class NetworkConsole {
public:
    NetworkConsole(int port = 23);
    void begin();
    void print(const char* message);
    void println(const char* message);
    void print(String message);
    void println(String message);
    void printf(const char* format, ...);

private:
    int _port;
    WiFiServer* _server;
    WiFiClient _client;

    void handleClient();
    void setupOTA();
};

#endif
