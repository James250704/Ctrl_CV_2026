// ============================================================================
// Hub — Central relay node (RP2350)
//
// Per agent.md topology:
//   Hub ↔ Jetson：hardware UART0 (GP0 TX, GP1 RX), 921600 baud, COBS framing
//   Hub ↔ Move：  PIO UART SM0
//   Hub ↔ Shoot： PIO UART SM1
//   Hub ↔ Storage：PIO UART SM2
//
// Core allocation (agent.md §核心分配):
//   Core 0 (loop):  ChassisComms ↔ Jetson + route to PIO UART slaves
//   Core 1 (loop1): Reserved for future PID / high-speed relay
// ============================================================================
#include "ChassisComms.h"
#include <Arduino.h>

// ============================================================================
// ChassisComms singleton — handles UART0 ↔ Jetson
// ============================================================================
static ChassisComms& comms = ChassisComms::getInstance();

// ============================================================================
// Callback: called by ChassisComms for every valid ControlPacket from Jetson.
// Per agent.md: Hub forwards commands to the appropriate slave via PIO UART.
// ============================================================================
static void onJetsonFrame(const ControlPacket& pkt) {
    // TODO: Route packet to the correct PIO UART based on pkt.targetID
    //   if (pkt.targetID == TargetID::HUB) {
    //       // Handle locally
    //   } else if (pkt.targetID == TargetID::MOVE) {
    //       pioUartTx(SM0, pkt);
    //   } else if (pkt.targetID == TargetID::SHOOT) {
    //       pioUartTx(SM1, pkt);
    //   } else if (pkt.targetID == TargetID::STORAGE) {
    //       pioUartTx(SM2, pkt);
    //   } else if (pkt.targetID == TargetID::BROADCAST) {
    //       pioUartTx(SM0, pkt); pioUartTx(SM1, pkt); pioUartTx(SM2, pkt);
    //   }

    // Stub: toggle LED on command
    static uint8_t lastSeq = 0xFF;
    if (pkt.frameSeq != lastSeq) {
        lastSeq = pkt.frameSeq;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}

// ============================================================================
// Core 0: setup()
// ============================================================================
void setup() {
    // Initialize ChassisComms (CRC table, UART0, DMA ping-pong)
    comms.init();

    // Register frame callback for incoming Jetson commands
    comms.setFrameCallback(onJetsonFrame);

    // Status LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

// ============================================================================
// Core 0: loop() — poll ChassisComms RX/TX state machine
// ============================================================================
void loop() {
    comms.update();
    delayMicroseconds(20);
}

// ============================================================================
// Core 1: Reserved for high-speed relay / PID
// ============================================================================
void setup1() {
    // TODO: Initialize PIO UART SM0, SM1, SM2 for slave communication
}

void loop1() {
    // Reserved
}
