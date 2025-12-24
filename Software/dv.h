/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * Data Visualizer interface for debugging through PICkit debugger.
 * Provides printf-style debugging and structured data streaming
 * directly through the debugger interface without requiring UART.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DV_H
#define DV_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Data Visualizer debug output functions */
void dv_init(void);
void dv_printf(const char *format, ...);
void dv_debug_int(const char *label, int32_t value);
void dv_debug_float(const char *label, float value);
void dv_debug_hex(const char *label, uint32_t value);

/* Structured data streaming for graphs/plots */
void dv_send_graph_data(uint8_t stream_id, float value);
void dv_send_multi_graph_data(uint8_t stream_id, float value1, float value2, float value3);

/* Direct byte streaming for advanced usage */
void dv_send_bytes(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* DV_H */