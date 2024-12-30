#ifndef NETWORKING_H
#define NETWORKING_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include "config.h"
#include "link.h"

void setupNetworking();
void sendJSON(struct MyLink *p, const char *shortAddress, const char *host, uint16_t portNum);
void sendData(const char* data, size_t len);

#endif // NETWORKING_H
