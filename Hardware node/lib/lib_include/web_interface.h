#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include <WiFi.h>
#include <WebServer.h>

extern WebServer server;

void setupWebServer();
void handleRoot();
void handleConfig();
void handleNotFound();

#endif // WEB_INTERFACE_H
