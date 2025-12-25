/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Menu system: nested menus navigated by the rotary encoder and selected via
 * encoder button. Displays two lines on the LCD. Cascade menus show an
 * ellipsis "..." appended to the item text. Each submenu contains a "Back"
 * item to return to the previous level; selecting "Back" at top level closes
 * the menu and restores the normal display.
 */

#include "menu.h"
#include "config.h"
#include "encoder.h"
#include "gps.h"
#include "lcd.h"
#include "serial.h"
#include "date.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <xc.h>

/*
 * Menu item definition
 */
typedef struct menu_item {
    const char* text;
    const struct menu_item* submenu; /* NULL if leaf item */
} menu_item_t;

static const menu_item_t calibrate_menu[] = {
    {"VCO", NULL}, 
    {"DAC", NULL}, 
    {"Back", NULL}, 
    {NULL, NULL}};

static const menu_item_t gps_protocol_menu[] = {
    {"NMEA", NULL}, 
    {"UBX", NULL}, 
    {"RTCM", NULL}, 
    {"Back", NULL}, 
    {NULL, NULL}};

static const menu_item_t stopbits_menu[] = {
    {"0", NULL}, 
    {"1", NULL}, 
    {"2", NULL}, 
    {"Back", NULL}, 
    {NULL, NULL}};

static const menu_item_t parity_menu[] = {
    {"None", NULL}, 
    {"Even", NULL}, 
    {"Odd", NULL}, 
    {"Mark", NULL}, 
    {"Space", NULL},
    {"Back", NULL}, 
    {NULL, NULL}};

static const menu_item_t baud_menu[] = {
    {"4800", NULL},  
    {"9600", NULL},   
    {"19200", NULL},  
    {"38400", NULL},
    {"57600", NULL}, 
    {"115200", NULL}, 
    {"230400", NULL}, 
    {"460800", NULL},
    {"Back", NULL},  
    {NULL, NULL}};

static const menu_item_t serial_menu[] = {
    {"Ext Baud", NULL}, 
    {"Ext Parity", NULL}, 
    {"Back", NULL}, 
    {NULL, NULL}};

static const menu_item_t gps_menu[] = {
    {"GPS Baud Rate", baud_menu},
    {"GPS Stop bits", stopbits_menu},
    {"GPS Parity", parity_menu},
    {"GPS Protocol", gps_protocol_menu},
    {"Back", NULL},
    {NULL, NULL}};

static const menu_item_t vref_menu[] = {
    {"DAC", NULL}, 
    {"Internal", NULL}, 
    {"Back", NULL}, 
    {NULL, NULL}};

static const menu_item_t date_menu[] = { 
    {"TZ Mode", NULL}, 
    {"TZ Offset", NULL}, 
    {"Back", NULL}, 
    {NULL, NULL} };

static const menu_item_t settings_menu[] = {
    {"VRef", vref_menu}, 
    {"Date/Time", date_menu}, 
    {"GPS", gps_menu}, 
    {"Serial", serial_menu}, 
    {"Back", NULL}, 
    {NULL, NULL}};

static const menu_item_t main_menu[] = {
    {"Status", NULL}, 
    {"Settings...", settings_menu}, 
    {"Calibrate...", calibrate_menu}, 
    {"Close", NULL}, 
    {NULL, NULL}};

/*
 * Menu runtime state
 */
#define MENU_MAX_DEPTH 4
typedef struct {
    const menu_item_t* stack[MENU_MAX_DEPTH];
    uint8_t selection[MENU_MAX_DEPTH];  // current selection index at each depth
    uint8_t depth;                      // current depth in menu stack
    uint8_t active;                     // 0 = closed, 1 = open 
    uint8_t last_encoder_pos;           // last encoder position
    uint8_t last_button;                // last encoder button state
    uint16_t notify_ticks;              // temporary message duration in ticks (~10ms)
    char notify_msg[21];                // temporary message text
    uint8_t editing;                    // 0 = not editing, otherwise EDIT_*
    uint8_t edit_value;                 // current temporary value while editing
} menu_t;

// Global menu state
static menu_t menu;                     

/*
 * Edit field identifiers
 */
#define EDIT_NONE 0
#define EDIT_VREF 1
#define EDIT_GPS_BAUD 2
#define EDIT_STOPBITS 3
#define EDIT_PARITY 4
#define EDIT_GPS_PROTOCOL 5
#define EDIT_EXT_BAUD 6
#define EDIT_EXT_PARITY 7
#define EDIT_TZ_MODE 8
#define EDIT_TZ_OFFSET 9

// Timezone offsets span -12:00..+14:00 in 15-minute steps (105 values)
#define TZ_OFFSET_STEPS 105

/*
 * Helpers
 */
static uint8_t menu_count(const menu_item_t* m) {
    uint8_t c = 0;
    while (m && m[c].text)
        c++;
    return c;
}

/**
 * Draw the current menu state to the LCD.
 */
static void menu_draw(void) {
    const menu_item_t* cur = menu.stack[menu.depth - 1];
    uint8_t cnt = menu_count(cur);
    if (cnt == 0)
        return;

    uint8_t sel = menu.selection[menu.depth - 1] % cnt;
    const char* s0 = cur[sel].text;
    char line0[21];
    memset(line0, 0, sizeof(line0));

    /* Prefix with '>' to mark selection */
    line0[0] = '>';
    /* Copy text leaving room for ellipsis */
    size_t avail = 19; // leave first char for '>'
    strncpy(&line0[1], s0, avail);
    line0[20] = '\0';

    /* If cascade (has submenu) append ellipsis at end (overwrite last 3 chars) */
    if (cur[sel].submenu) {
        line0[17] = '.';
        line0[18] = '.';
        line0[19] = '.';
    }

    /* Prepare second line: show following menu item (cyclic) */
    const char* s1 = cur[(sel + 1) % cnt].text;
    char line1[21];
    for (size_t i = 0; i < 20; ++i)
        line1[i] = ' ';
    line1[20] = '\0';
    size_t copy = strlen(s1);
    if (copy > 20)
        copy = 20;
    memcpy(line1, s1, copy);

    lcdBufferSetLine(0, line0);
    lcdBufferSetLine(1, line1);
}

/**
 * Initialize menu system
 */
void menu_init(void) {
    memset(&menu, 0, sizeof(menu));
    menu.last_encoder_pos = encoder_get_position();
    menu.last_button = encoder_button_state();
    menu.active = 0;
    menu.notify_ticks = 0;
}

/**
 * Open the menu system
 */
void menu_open(void) {
    menu.depth = 1;
    menu.stack[0] = main_menu;
    menu.selection[0] = 0;
    menu.active = 1;
    menu.last_encoder_pos = encoder_get_position();
    menu.last_button = encoder_button_state();
    menu.editing = EDIT_NONE;
    menu.edit_value = 0;
    lcdBufferClear();
    menu_draw();
}

/**
 * Close the menu system
 */
void menu_close(void) {
    menu.active = 0;
    /* Restore startup display */
    updateDisplay();
}

/*
 * Display a temporary message for `duration_ms` milliseconds
 */
static void menu_show_message(const char* msg, uint16_t duration_ms) {
    if (!msg)
        return;
    strncpy(menu.notify_msg, msg, 20);
    menu.notify_msg[20] = '\0';
    menu.notify_ticks = (duration_ms + 9) / 10; /* convert to ticks */
    lcdBufferClear();
    lcdBufferSetLine(0, menu.notify_msg);
}

/**
 * Process menu input and state (to be called periodically)
 */
void menu_process(void) {
    /* Handle temporary message timeout */
    if (menu.notify_ticks) {
        if (--menu.notify_ticks == 0) {
            menu_close();
        }
        return;
    }

    uint8_t cur_pos = encoder_get_position();
    int8_t delta = (int8_t)(cur_pos - menu.last_encoder_pos);
    if (delta) {
        menu.last_encoder_pos = cur_pos;
        if (menu.active) {
            if (menu.editing != EDIT_NONE) {
                /* editing a field: adjust edit_value */
                uint8_t max = 1;
                switch (menu.editing) {
                    case EDIT_VREF:
                        max = 2;
                        break;
                    case EDIT_GPS_BAUD:
                        max = 8;
                        break;
                    case EDIT_STOPBITS:
                        max = 3;
                        break;
                    case EDIT_PARITY:
                        max = 3;
                        break;
                    case EDIT_GPS_PROTOCOL:
                        max = 3;
                        break;
                    case EDIT_EXT_BAUD:
                        max = BAUD_RATES_COUNT;
                        break;
                    case EDIT_EXT_PARITY:
                        max = PARITY_OPTIONS_COUNT;
                        break;
                    case EDIT_TZ_MODE:
                        max = 2;
                        break;
                    case EDIT_TZ_OFFSET:
                        max = TZ_OFFSET_STEPS;
                        break;
                }
                uint8_t v = (uint8_t)(menu.edit_value + delta);
                v %= max;
                menu.edit_value = v;
                /* show edit state */
                char buf[21];
                memset(buf, ' ', sizeof(buf));
                buf[20] = '\0';
                switch (menu.editing) {
                    case EDIT_VREF:
                        memcpy(buf, "VRef:", 5);
                        memcpy(&buf[6], vref_options[menu.edit_value], strlen(vref_options[menu.edit_value]));
                        break;
                    case EDIT_GPS_BAUD:
                        memcpy(buf, "GPS Baud:", 9);
                        memcpy(&buf[10], baud_options[menu.edit_value], strlen(baud_options[menu.edit_value]));
                        break;
                    case EDIT_STOPBITS:
                        memcpy(buf, "Stop bits:", 10);
                        memcpy(&buf[11], stop_options[menu.edit_value], strlen(stop_options[menu.edit_value]));
                        break;
                    case EDIT_PARITY:
                        memcpy(buf, "Parity:", 7);
                        memcpy(&buf[8], parity_options[menu.edit_value], strlen(parity_options[menu.edit_value]));
                        break;
                }
                buf[20] = '\0';
                lcdBufferClear();
                lcdBufferSetLine(0, buf);
                lcdBufferSetLine(1, "Press to save");
            } else {
                const menu_item_t* cur = menu.stack[menu.depth - 1];
                uint8_t cnt = menu_count(cur);
                if (cnt > 0) {
                    uint8_t sel = menu.selection[menu.depth - 1];
                    /* wrap using modulo arithmetic */
                    sel = (uint8_t)(sel + delta);
                    /* Ensure 0..cnt-1 */
                    sel %= cnt;
                    menu.selection[menu.depth - 1] = sel;
                    menu_draw();
                }
            }
        }
    }

    uint8_t btn = encoder_button_state();
    if (btn && !menu.last_button) {
        /* Button pressed event */
        if (!menu.active) {
            menu_open();
        } else {
            /* Act on selected item */
            const menu_item_t* cur = menu.stack[menu.depth - 1];
            uint8_t sel = menu.selection[menu.depth - 1];
            const menu_item_t* item = &cur[sel];

            /* Check for Back/Close by text matching (simple) */
            if (strcmp(item->text, "Back") == 0 || strcmp(item->text, "Close") == 0) {
                if (menu.depth > 1) {
                    menu.depth--;
                    lcdBufferClear();
                    menu_draw();
                } else {
                    menu_close();
                }
            } else if (item->submenu) {
                /* Enter submenu */
                if (menu.depth < MENU_MAX_DEPTH) {
                    menu.stack[menu.depth] = item->submenu;
                    menu.selection[menu.depth] = 0;
                    menu.depth++;
                    lcdBufferClear();
                    menu_draw();
                }
            } else {
                /* Leaf: if we're in settings menu, switch to edit mode for known items */
                if (menu.stack[menu.depth - 1] == settings_menu) {
                    switch (sel) {
                        case 0: /* VRef */
                            menu.editing = EDIT_VREF;
                            menu.edit_value = (uint8_t)system_config.vref_source;
                            break;
                        default:
                            /* For other leaves, show selection message */
                            {
                                char buf[21];
                                const char prefix[] = "Selected: ";
                                size_t p = sizeof(prefix) - 1;
                                memset(buf, ' ', sizeof(buf));
                                if (p < 20) {
                                    memcpy(buf, prefix, p);
                                    size_t tlen = strlen(item->text);
                                    size_t copy = (tlen > (20 - p)) ? (20 - p) : tlen;
                                    memcpy(&buf[p], item->text, copy);
                                } else {
                                    memcpy(buf, prefix, 20);
                                }
                                buf[20] = '\0';
                                menu_show_message(buf, 1000);
                            }
                            break;
                    }
                } else if (menu.stack[menu.depth - 1] == gps_menu) {
                    switch (sel) {
                        case 0: /* GPS Baud */
                            menu.editing = EDIT_GPS_BAUD;
                            menu.edit_value = system_config.gps_baud_index;
                            break;
                        case 1: /* GPS Stop bits */
                            menu.editing = EDIT_STOPBITS;
                            menu.edit_value = system_config.gps_stop_bits;
                            break;
                        case 2: /* GPS Parity */
                            menu.editing = EDIT_PARITY;
                            menu.edit_value = system_config.gps_parity;
                            break;
                        case 3: /* GPS Protocol */
                            menu.editing = EDIT_GPS_PROTOCOL;
                            menu.edit_value = system_config.gps_protocol;
                            break;
                        default:
                            /* For other leaves, show selection message */
                            {
                                char buf[21];
                                const char prefix[] = "Selected: ";
                                size_t p = sizeof(prefix) - 1;
                                memset(buf, ' ', sizeof(buf));
                                if (p < 20) {
                                    memcpy(buf, prefix, p);
                                    size_t tlen = strlen(item->text);
                                    size_t copy = (tlen > (20 - p)) ? (20 - p) : tlen;
                                    memcpy(&buf[p], item->text, copy);
                                } else {
                                    memcpy(buf, prefix, 20);
                                }
                                buf[20] = '\0';
                                menu_show_message(buf, 1000);
                            }
                            break;
                    }
                } else if (menu.stack[menu.depth - 1] == date_menu) {
                    switch (sel) {
                        case 0: /* TZ Mode */
                            menu.editing = EDIT_TZ_MODE;
                            menu.edit_value = system_config.tz_mode;
                            break;
                        case 1: /* TZ Offset */
                            menu.editing = EDIT_TZ_OFFSET;
                            /* Map offset (-720..+840) in 15-minute steps to an index */
                            menu.edit_value = (uint8_t)((system_config.tz_offset_min + 720) / 15);
                            break;
                        default:
                            /* For other leaves, show selection message */
                            {
                                char buf[21];
                                const char prefix[] = "Selected: ";
                                size_t p = sizeof(prefix) - 1;
                                memset(buf, ' ', sizeof(buf));
                                if (p < 20) {
                                    memcpy(buf, prefix, p);
                                    size_t tlen = strlen(item->text);
                                    size_t copy = (tlen > (20 - p)) ? (20 - p) : tlen;
                                    memcpy(&buf[p], item->text, copy);
                                } else {
                                    memcpy(buf, prefix, 20);
                                }
                                buf[20] = '\0';
                                menu_show_message(buf, 1000);
                            }
                            break;
                    }
                } else if (menu.stack[menu.depth - 1] == serial_menu) {
                    switch (sel) {
                        case 0: /* External Baud Rate */
                            menu.editing = EDIT_EXT_BAUD;
                            menu.edit_value = system_config.ext_baud_index;
                            break;
                        case 1: /* External Parity */
                            menu.editing = EDIT_EXT_PARITY;
                            menu.edit_value = system_config.ext_parity;
                            break;
                        default:
                            /* For other leaves, show selection message */
                            {
                                char buf[21];
                                const char prefix[] = "Selected: ";
                                size_t p = sizeof(prefix) - 1;
                                memset(buf, ' ', sizeof(buf));
                                if (p < 20) {
                                    memcpy(buf, prefix, p);
                                    size_t tlen = strlen(item->text);
                                    size_t copy = (tlen > (20 - p)) ? (20 - p) : tlen;
                                    memcpy(&buf[p], item->text, copy);
                                } else {
                                    memcpy(buf, prefix, 20);
                                }
                                buf[20] = '\0';
                                menu_show_message(buf, 1000);
                            }
                            break;
                    }

                    /* Show edit display */
                    if (menu.editing != EDIT_NONE) {
                        /* Render immediate edit view */
                        char buf[21];
                        memset(buf, ' ', sizeof(buf));
                        buf[20] = '\0';
                        switch (menu.editing) {
                            case EDIT_VREF:
                                memcpy(buf, "VRef:", 5);
                                memcpy(&buf[6], vref_options[menu.edit_value], strlen(vref_options[menu.edit_value]));
                                break;
                            case EDIT_GPS_BAUD:
                                memcpy(buf, "GPS Baud:", 9);
                                memcpy(&buf[10], baud_options[menu.edit_value], strlen(baud_options[menu.edit_value]));
                                break;
                            case EDIT_STOPBITS:
                                memcpy(buf, "GPS Stop:", 9);
                                memcpy(&buf[10], stop_options[menu.edit_value], strlen(stop_options[menu.edit_value]));
                                break;
                            case EDIT_PARITY:
                                memcpy(buf, "GPS Parity:", 11);
                                memcpy(&buf[12], parity_options[menu.edit_value],
                                       strlen(parity_options[menu.edit_value]));
                                break;
                            case EDIT_GPS_PROTOCOL:
                                memcpy(buf, "GPS Proto:", 10);
                                memcpy(&buf[11], protocol_options[menu.edit_value],
                                       strlen(protocol_options[menu.edit_value]));
                                break;
                            case EDIT_EXT_BAUD:
                                memcpy(buf, "Ext Baud:", 9);
                                memcpy(&buf[10], baud_options[menu.edit_value], strlen(baud_options[menu.edit_value]));
                                break;
                            case EDIT_EXT_PARITY:
                                memcpy(buf, "Ext Parity:", 11);
                                memcpy(&buf[12], parity_options[menu.edit_value],
                                       strlen(parity_options[menu.edit_value]));
                                break;
                            case EDIT_TZ_MODE:
                                memcpy(buf, "TZ Mode:", 8);
                                memcpy(&buf[9], tz_mode_options[menu.edit_value], strlen(tz_mode_options[menu.edit_value]));
                                break;
                            case EDIT_TZ_OFFSET: {
                                char ofs[8];
                                int16_t min = (int16_t)menu.edit_value * 15 - 720;
                                tz_offset_to_string(min, ofs);
                                memcpy(buf, "TZ Offset:", 11);
                                memcpy(&buf[12], ofs, strlen(ofs));
                                break;
                            }
                        }
                        lcdBufferClear();
                        lcdBufferSetLine(0, buf);
                        lcdBufferSetLine(1, "Press to save");
                        /* consume this button event by returning now so save requires
                         * a subsequent press (avoids entering-and-saving on same press) */
                        menu.last_button = btn;
                        return;
                    }
                } else {
                    /* Leaf item: show temporary selection message then close */
                    char buf[21];
                    const char prefix[] = "Selected: ";
                    size_t p = sizeof(prefix) - 1; /* excludes null */
                    memset(buf, ' ', sizeof(buf));
                    if (p < 20) {
                        memcpy(buf, prefix, p);
                        size_t tlen = strlen(item->text);
                        size_t copy = (tlen > (20 - p)) ? (20 - p) : tlen;
                        memcpy(&buf[p], item->text, copy);
                    } else {
                        memcpy(buf, prefix, 20);
                    }
                    buf[20] = '\0';
                    menu_show_message(buf, 1000);
                }
            }
        }
    }
    /* Handle saving while in edit mode and button pressed */
    if (menu.editing != EDIT_NONE && btn && !menu.last_button) {
        switch (menu.editing) {
            case EDIT_VREF:
                system_config.vref_source = menu.edit_value;
                break;
            case EDIT_GPS_BAUD:
                system_config.gps_baud_index = menu.edit_value;
                break;
            case EDIT_STOPBITS:
                system_config.gps_stop_bits = menu.edit_value;
                break;
            case EDIT_PARITY:
                system_config.gps_parity = menu.edit_value;
                break;
            case EDIT_GPS_PROTOCOL:
                system_config.gps_protocol = menu.edit_value;
                gps_set_protocol((gps_protocol_t)menu.edit_value);
                break;
            case EDIT_EXT_BAUD:
                system_config.ext_baud_index = menu.edit_value;
                serial_reconfigure(); // Apply new settings immediately
                break;
            case EDIT_EXT_PARITY:
                system_config.ext_parity = menu.edit_value;
                serial_reconfigure(); // Apply new settings immediately
                break;
            case EDIT_TZ_MODE:
                system_config.tz_mode = menu.edit_value;
                break;
            case EDIT_TZ_OFFSET:
                /* map index back to minutes */
                system_config.tz_offset_min = (int16_t)menu.edit_value * 15 - 720;
                break;
        }
        config_save((const system_config_t*)&system_config);
        menu_show_message("Saved", 800);
        menu.editing = EDIT_NONE;
        menu_draw();
    }
    menu.last_button = btn;
}
