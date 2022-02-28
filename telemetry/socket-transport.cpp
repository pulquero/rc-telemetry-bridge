#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>

#include "protocol.h"
#include "socket-transport.h"
#include "debug.h"

#define SOCKET_BUFFER_SIZE 64
#define MAX_CLIENTS 2
#define SOCKET_RECONNECT_DELAY 2000

static Telemetry* _telemetry;
static AsyncClient* inputClient = nullptr;
static AsyncServer* socketServer = nullptr;

static AsyncClient* outputClients[MAX_CLIENTS];

void socketBegin(Telemetry* telemetry) {
  _telemetry = telemetry;
  if (!inputClient && telemetry->config.input.source == SOURCE_SOCKET && strlen(telemetry->config.socket.client.hostname)) {
    inputClient = new AsyncClient();
    inputClient->setNoDelay(true);
    inputClient->onData([](void* arg, AsyncClient* client, void* data, size_t len) {
      _telemetry->copyToIncoming((uint8_t*)data, len, SOURCE_SOCKET);
    }, NULL);
  }
  if (!socketServer && telemetry->config.socket.server.mode != MODE_DISABLED) {
    socketServer = new AsyncServer(telemetry->config.socket.server.port);
    socketServer->onClient([](void* arg, AsyncClient* client) {
      for (int i=0; i<MAX_CLIENTS; i++) {
        if (!outputClients[i]) {
          outputClients[i] = client;
          return;
        }
      }
      client->stop();
      delete client;
    }, NULL);
    if (telemetry->config.socket.server.mode == MODE_FILTER) {
      socketServer->setNoDelay(true);
    } else {
      socketServer->setNoDelay(false);
    }
    socketServer->begin();
    LOGD("Started socket server on port %d", telemetry->config.socket.server.port);
  }
}

void socketStop() {
  if (inputClient) {
    inputClient->stop();
    delete inputClient;
    inputClient = nullptr;
  }
  if (socketServer) {
    for (int i=0; i<MAX_CLIENTS; i++) {
      AsyncClient* client = outputClients[i];
      if (client) {
        client->stop();
        delete client;
        outputClients[i] = nullptr;
      }
    }
    socketServer->end();
    delete socketServer;
    socketServer = nullptr;
  }
  _telemetry = nullptr;
}

void socketWrite(uint8_t* data, int len) {
  for (int i=0; i<MAX_CLIENTS; i++) {
    AsyncClient* client = outputClients[i];
    if (client) {
      if (client->connected()) {
        if (client->canSend()) {
          client->write((char*)data, len);
        }
      } else if (client->disconnected()) {
        client->stop();
        delete client;
        outputClients[i] = nullptr;
      }
    }
  }
}

void socketEnsureConnected(uint32_t ms) {
  static uint32_t lastConnectAttempt = 0;
  if (inputClient && !inputClient->connected() && !inputClient->connecting() && (ms - lastConnectAttempt) > SOCKET_RECONNECT_DELAY) {
    LOGD("Trying to connect to socket %s:%d", _telemetry->config.socket.client.hostname, _telemetry->config.socket.client.port);
    bool ok = inputClient->connect(_telemetry->config.socket.client.hostname, _telemetry->config.socket.client.port);
    if (ok) {
      LOGD("Socket client connection: %s", inputClient->stateToString());
      lastConnectAttempt = 0;
    } else {
      LOGD("Socket client connection error");
      lastConnectAttempt = ms;
    }
  }
}
