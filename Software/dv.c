/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * Data Visualizer DVRT implementation over PICkit debugger interface.
 * Uses specific memory locations and packet format for MPLAB Data Visualizer.
 */

#include "dv.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* DVRT Protocol Constants */
#define DVRT_START_BYTE1 0x55
#define DVRT_START_BYTE2 0xAA
#define DVRT_PACKET_TEXT 0x01
#define DVRT_PACKET_DATA 0x02
#define DVRT_PACKET_GRAPH 0x03

/* DVRT Communication through debugger interface */
/* These memory locations are monitored by the PICkit debugger */
static volatile uint8_t dvrt_tx_buffer[64];
static volatile uint8_t dvrt_tx_length = 0;
static volatile uint8_t dvrt_tx_ready = 0;
static volatile uint16_t dvrt_sequence = 0;

/*
 * Initialize Data Visualizer DVRT interface
 */
void dv_init(void) {
    // Clear DVRT buffers
    memset((void*)dvrt_tx_buffer, 0, sizeof(dvrt_tx_buffer));
    dvrt_tx_length = 0;
    dvrt_tx_ready = 0;
    dvrt_sequence = 0;
}

/*
 * Send DVRT packet through debugger interface
 * Data Visualizer monitors these memory locations
 */
static void dvrt_send_packet(uint8_t packet_type, const uint8_t* data, uint8_t data_length) {
    if (data_length > 58)
        return; // Max payload: 64 - 6 header bytes

    // Wait for previous packet to be processed
    while (dvrt_tx_ready) { /* Wait */
    }

    // Build DVRT packet in buffer
    uint8_t index = 0;
    dvrt_tx_buffer[index++] = DVRT_START_BYTE1;
    dvrt_tx_buffer[index++] = DVRT_START_BYTE2;
    dvrt_tx_buffer[index++] = packet_type;
    dvrt_tx_buffer[index++] = data_length;
    dvrt_tx_buffer[index++] = (uint8_t)(dvrt_sequence & 0xFF);
    dvrt_tx_buffer[index++] = (uint8_t)((dvrt_sequence >> 8) & 0xFF);

    // Copy payload
    for (uint8_t i = 0; i < data_length; i++) {
        dvrt_tx_buffer[index++] = data[i];
    }

    // Set packet ready
    dvrt_tx_length = index;
    dvrt_sequence++;
    dvrt_tx_ready = 1; // Signal to Data Visualizer that packet is ready
}

/*
 * Printf-style debug output to Data Visualizer
 */
void dv_printf(const char* format, ...) {
    char debug_buffer[58]; // Max DVRT payload size
    va_list args;

    va_start(args, format);
    int len = vsnprintf(debug_buffer, sizeof(debug_buffer), format, args);
    va_end(args);

    if (len > 0 && len < sizeof(debug_buffer)) {
        dvrt_send_packet(DVRT_PACKET_TEXT, (uint8_t*)debug_buffer, (uint8_t)len);
    }
}

/*
 * Debug helper for integer values
 */
void dv_debug_int(const char* label, int32_t value) {
    char debug_buffer[58];
    int len = snprintf(debug_buffer, sizeof(debug_buffer), "[DV] %s: %ld\r\n", label, value);
    if (len > 0 && len < sizeof(debug_buffer)) {
        dvrt_send_packet(DVRT_PACKET_TEXT, (uint8_t*)debug_buffer, (uint8_t)len);
    }
}

/*
 * Debug helper for float values
 */
void dv_debug_float(const char* label, float value) {
    char debug_buffer[58];
    int len = snprintf(debug_buffer, sizeof(debug_buffer), "[DV] %s: %.6f\r\n", label, value);
    if (len > 0 && len < sizeof(debug_buffer)) {
        dvrt_send_packet(DVRT_PACKET_TEXT, (uint8_t*)debug_buffer, (uint8_t)len);
    }
}

/*
 * Debug helper for hexadecimal values
 */
void dv_debug_hex(const char* label, uint32_t value) {
    char debug_buffer[58];
    int len = snprintf(debug_buffer, sizeof(debug_buffer), "[DV] %s: 0x%08lX\r\n", label, value);
    if (len > 0 && len < sizeof(debug_buffer)) {
        dvrt_send_packet(DVRT_PACKET_TEXT, (uint8_t*)debug_buffer, (uint8_t)len);
    }
}

/*
 * Send single value for graphing using DVRT protocol
 */
void dv_send_graph_data(uint8_t stream_id, float value) {
    uint8_t packet_data[5];
    packet_data[0] = stream_id;

    // Copy float as bytes
    uint8_t* float_bytes = (uint8_t*)&value;
    for (int i = 0; i < 4; i++) {
        packet_data[i + 1] = float_bytes[i];
    }

    dvrt_send_packet(DVRT_PACKET_GRAPH, packet_data, 5);
}

/*
 * Send multiple values for multi-line graphing
 */
void dv_send_multi_graph_data(uint8_t stream_id, float value1, float value2, float value3) {
    uint8_t packet_data[13];
    packet_data[0] = stream_id;

    // Copy first float
    uint8_t* bytes1 = (uint8_t*)&value1;
    for (int i = 0; i < 4; i++)
        packet_data[i + 1] = bytes1[i];

    // Copy second float
    uint8_t* bytes2 = (uint8_t*)&value2;
    for (int i = 0; i < 4; i++)
        packet_data[i + 5] = bytes2[i];

    // Copy third float
    uint8_t* bytes3 = (uint8_t*)&value3;
    for (int i = 0; i < 4; i++)
        packet_data[i + 9] = bytes3[i];

    dvrt_send_packet(DVRT_PACKET_GRAPH, packet_data, 13);
}

/*
 * Send raw byte array to Data Visualizer
 */
void dv_send_bytes(const uint8_t* data, uint16_t length) {
    // Send in chunks if data is too large
    uint16_t offset = 0;
    while (offset < length) {
        uint16_t chunk_size = (length - offset > 58) ? 58 : (length - offset);
        dvrt_send_packet(DVRT_PACKET_DATA, data + offset, (uint8_t)chunk_size);
        offset += chunk_size;
    }
}