/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Menu system: nested menus navigated by the rotary encoder and selected via
 * encoder button. Displays four lines on the LCD. Cascade menus show an
 * ellipsis "..." appended to the item text. Each submenu contains a "Back"
 * item to return to the previous level; selecting "Back" at top level closes
 * the menu and restores the normal display.
 * 
 * The menu data strucures are defined as static constant arrays of menu_item_t
 * structures. Each menu item has display text and an optional pointer to a
 * submenu (NULL if leaf item).  As menu items are selected, the "value" of the
 * item is stored in the global configuration structure and saved to non-volatile 
 * memory.
 */

#include "menu.h"
#include "config.h"
#include "date.h"
#include "encoder.h"
#include "faults.h"
#include "gps.h"
#include "lcd.h"
#include "serial.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <xc.h>

/*
 * Define the structure of a menu item.
 */
typedef struct menu_item {
    const char* text;                /* Display text */
    const struct menu_item* submenu; /* NULL if leaf item */
} menu_item_t;

/*
 * The GPS protocol menu allows a user to select the GPS protocol to be processed.  The protocol can be 
 * NMEA, UBX, or RTCM.  When the protocol is changed, the GPS module is reconfigured to output only the 
 * selected protocol.
 */
static const menu_item_t gps_protocol_menu[] = {
    {"NMEA", NULL},
    {"UBX", NULL}, 
    {"RTCM", NULL}, 
    {"Back", NULL}, 
    {NULL, NULL}};

/*
 * The stop bits menu allows a user to select the number of stop bits used in serial communication.
 * Supported stop bits are 1, 1.5, and 2.
 */
static const menu_item_t stopbits_menu[] = {
    {"1", NULL},    
    {"1.5", NULL},  
    {"2", NULL},    
    {"Back", NULL}, 
    {NULL, NULL}};

/*
 * The parity menu allows a user to select the parity mode used in serial communication.
 * Supported parity modes are None, Even, Odd, Mark, and Space.
 */
static const menu_item_t parity_menu[] = {
    {"None", NULL},  
    {"Even", NULL}, 
    {"Odd", NULL}, 
    {"Mark", NULL},
    {"Space", NULL}, 
    {"Back", NULL}, 
    {NULL, NULL}};

/*                                          
 * The baud rate menu allows a user to select the baud rate used in serial communication.
 * Supported baud rates range from 300 to 460800.
 */
static const menu_item_t baud_menu[] = {
    {"300", NULL},   
    {"600", NULL},    
    {"1200", NULL},   
    {"2400", NULL},
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

/*
 * The "External" serial port is the data port that the GPSDO uses to send location and date/time 
 * information to any downstream application(s).  This can be configured separately from the 
 * GPS serial interface.
 */                                        
static const menu_item_t serial_menu[] = {
    {"Baud rate...", baud_menu}, 
    {"Stop bits...", stopbits_menu}, 
    {"Parity...", parity_menu}, 
    {"Back", NULL}, 
    {NULL, NULL}};

/*
 * The GPS menu provides access to configuration options for the GPS module.
 */
static const menu_item_t gps_menu[] = {
    {"Baud Rate...", baud_menu}, 
    {"Stop bits...", stopbits_menu}, 
    {"Parity...", parity_menu},
    {"Protocol...", gps_protocol_menu},  
    {"Back", NULL}, 
    {NULL, NULL}};

/*
 * The timezone offset menu allows selection of timezone offsets from -12:00 to +14:00 in 15-minute increments.
 * Range includes all real-world timezones (e.g., UTC-12:00 in Baker Island to UTC+14:00 in Kiribati).
 */
static const menu_item_t offset_menu[] = {
    {"-12:00", NULL}, {"-11:45", NULL}, {"-11:30", NULL}, {"-11:15", NULL}, {"-11:00", NULL},
    {"-10:45", NULL}, {"-10:30", NULL}, {"-10:15", NULL}, {"-10:00", NULL}, {"-9:45", NULL},
    {"-9:30", NULL},  {"-9:15", NULL},  {"-9:00", NULL},  {"-8:45", NULL},  {"-8:30", NULL},
    {"-8:15", NULL},  {"-8:00", NULL},  {"-7:45", NULL},  {"-7:30", NULL},  {"-7:15", NULL},
    {"-7:00", NULL},  {"-6:45", NULL},  {"-6:30", NULL},  {"-6:15", NULL},  {"-6:00", NULL},
    {"-5:45", NULL},  {"-5:30", NULL},  {"-5:15", NULL},  {"-5:00", NULL},  {"-4:45", NULL},
    {"-4:30", NULL},  {"-4:15", NULL},  {"-4:00", NULL},  {"-3:45", NULL},  {"-3:30", NULL},
    {"-3:15", NULL},  {"-3:00", NULL},  {"-2:45", NULL},  {"-2:30", NULL},  {"-2:15", NULL},
    {"-2:00", NULL},  {"-1:45", NULL},  {"-1:30", NULL},  {"-1:15", NULL},  {"-1:00", NULL},
    {"-0:45", NULL},  {"-0:30", NULL},  {"-0:15", NULL},  {"+0:00", NULL},  {"+0:15", NULL},
    {"+0:30", NULL},  {"+0:45", NULL},  {"+1:00", NULL},  {"+1:15", NULL},  {"+1:30", NULL},
    {"+1:45", NULL},  {"+2:00", NULL},  {"+2:15", NULL},  {"+2:30", NULL},  {"+2:45", NULL},
    {"+3:00", NULL},  {"+3:15", NULL},  {"+3:30", NULL},  {"+3:45", NULL},  {"+4:00", NULL},
    {"+4:15", NULL},  {"+4:30", NULL},  {"+4:45", NULL},  {"+5:00", NULL},  {"+5:15", NULL},
    {"+5:30", NULL},  {"+5:45", NULL},  {"+6:00", NULL},  {"+6:15", NULL},  {"+6:30", NULL},
    {"+6:45", NULL},  {"+7:00", NULL},  {"+7:15", NULL},  {"+7:30", NULL},  {"+7:45", NULL},
    {"+8:00", NULL},  {"+8:15", NULL},  {"+8:30", NULL},  {"+8:45", NULL},  {"+9:00", NULL},
    {"+9:15", NULL},  {"+9:30", NULL},  {"+9:45", NULL},  {"+10:00", NULL}, {"+10:15", NULL},
    {"+10:30", NULL}, {"+10:45", NULL}, {"+11:00", NULL}, {"+11:15", NULL}, {"+11:30", NULL},
    {"+11:45", NULL}, {"+12:00", NULL}, {"+12:15", NULL}, {"+12:30", NULL}, {"+12:45", NULL},
    {"+13:00", NULL}, {"+13:15", NULL}, {"+13:30", NULL}, {"+13:45", NULL}, {"+14:00", NULL},
    {"Back", NULL},
    {NULL, NULL}};

/*
 * Allows selection of timezone mode: UTC or Local.  If local is selected, the timezone offset
 * is applied to convert UTC time from the GPS to local time.
 */
static const menu_item_t timezone_menu[] = {
    {"UTC", NULL}, 
    {"Local...", offset_menu}, 
    {"Back", NULL}, 
    {NULL, NULL}};

/*
 * The "Date/Time" menu provides access to date and time configuration options.
 */
static const menu_item_t date_menu[] = {
    {"Timezone...", timezone_menu}, 
    {"Interval", NULL},
    {"Back", NULL}, 
    {NULL, NULL}};

/*
 * The "Faults" menu allows viewing of up to 5 recent fault messages.
 * Selecting a fault number displays the fault details.
 */
static const menu_item_t faults_menu[] = {
    {"Fault 1", NULL},
    {"Fault 2", NULL},
    {"Fault 3", NULL},
    {"Fault 4", NULL},
    {"Fault 5", NULL},
    {"Clear All", NULL},
    {"Back", NULL},
    {NULL, NULL}};

/**
 * The main menu is the top-level menu presented to the user.  It provides access to
 * the GPS configuration, External serial port configuration, and OCXO calibration.
 */
static const menu_item_t main_menu[] = {
    {"GPS...", gps_menu}, 
    {"Serial...", serial_menu}, 
    {"Date/Time...", date_menu},
    {"Faults...", faults_menu},
    {"Close", NULL}, 
    {NULL, NULL}};

/*
 * Menu runtime state
 */
#define MENU_MAX_DEPTH 4
typedef struct {
    const menu_item_t* stack[MENU_MAX_DEPTH];
    unsigned char selection[MENU_MAX_DEPTH]; // current selection index at each depth
    unsigned char depth;                     // current depth in menu stack
    unsigned char active;                    // 0 = closed, 1 = open
    unsigned char last_encoder_pos;          // last encoder position
    unsigned char last_button;               // last encoder button state
    unsigned int notify_ticks;             // temporary message duration in ticks (~10ms)
    char notify_msg[21];               // temporary message text
    unsigned char editing;                   // 0 = not editing, otherwise EDIT_*
    unsigned char edit_value;                // current temporary value while editing
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
static unsigned char menu_count(const menu_item_t* m) {
    unsigned char c = 0;
    while (m && m[c].text)
        c++;
    return c;
}

/* Get menu item text by index */
static const char* menu_get_text(const menu_item_t* m, unsigned char idx) {
    if (m && m[idx].text)
        return m[idx].text;
    return "";
}

/* Convert baud rate string to numeric value */
static long baud_str_to_value(const char* str) {
    long val = 0;
    while (*str >= '0' && *str <= '9') {
        val = val * 10 + (*str - '0');
        str++;
    }
    return val;
}

/* Find index of baud rate value in baud_menu */
static unsigned char baud_value_to_index(long baud) {
    unsigned char count = menu_count(baud_menu) - 1; // Exclude "Back"
    for (unsigned char i = 0; i < count; i++) {
        if (baud_str_to_value(baud_menu[i].text) == baud)
            return i;
    }
    return 5; // Default to 9600
}

/**
 * Draw the current menu state to the LCD.
 */
static void menu_draw(void) {
    const menu_item_t* cur = menu.stack[menu.depth - 1];
    unsigned char cnt = menu_count(cur);
    if (cnt == 0)
        return;

    unsigned char sel = menu.selection[menu.depth - 1] % cnt;
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
static void menu_show_message(const char* msg, unsigned int duration_ms) {
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

    unsigned char cur_pos = encoder_get_position();
    unsigned char delta = (unsigned char)(cur_pos - menu.last_encoder_pos);
    if (delta) {
        menu.last_encoder_pos = cur_pos;
        if (menu.active) {
            if (menu.editing != EDIT_NONE) {
                /* editing a field: adjust edit_value */
                unsigned char max = 1;
                switch (menu.editing) {
                    case EDIT_VREF:
                        max = 2; // Internal, External
                        break;
                    case EDIT_GPS_BAUD:
                        max = menu_count(baud_menu) - 1; // Exclude "Back"
                        break;
                    case EDIT_STOPBITS:
                        max = menu_count(stopbits_menu) - 1;
                        break;
                    case EDIT_PARITY:
                        max = menu_count(parity_menu) - 1;
                        break;
                    case EDIT_GPS_PROTOCOL:
                        max = menu_count(gps_protocol_menu) - 1;
                        break;
                    case EDIT_EXT_BAUD:
                        max = menu_count(baud_menu) - 1;
                        break;
                    case EDIT_EXT_PARITY:
                        max = menu_count(parity_menu) - 1;
                        break;
                    case EDIT_TZ_MODE:
                        max = menu_count(timezone_menu) - 1;
                        break;
                    case EDIT_TZ_OFFSET:
                        max = TZ_OFFSET_STEPS;
                        break;
                }
                unsigned char v = (unsigned char)(menu.edit_value + delta);
                v %= max;
                menu.edit_value = v;
                /* show edit state */
                char buf[21];
                memset(buf, ' ', sizeof(buf));
                buf[20] = '\0';
                const char* text;
                switch (menu.editing) {
                    case EDIT_VREF:
                        text = menu.edit_value == 0 ? "Internal" : "External";
                        memcpy(buf, "VRef:", 5);
                        memcpy(&buf[6], text, strlen(text));
                        break;
                    case EDIT_GPS_BAUD:
                        text = menu_get_text(baud_menu, menu.edit_value);
                        memcpy(buf, "GPS Baud:", 9);
                        memcpy(&buf[10], text, strlen(text));
                        break;
                    case EDIT_STOPBITS:
                        text = menu_get_text(stopbits_menu, menu.edit_value);
                        memcpy(buf, "Stop bits:", 10);
                        memcpy(&buf[11], text, strlen(text));
                        break;
                    case EDIT_PARITY:
                        text = menu_get_text(parity_menu, menu.edit_value);
                        memcpy(buf, "Parity:", 7);
                        memcpy(&buf[8], text, strlen(text));
                        break;
                }
                buf[20] = '\0';
                lcdBufferClear();
                lcdBufferSetLine(0, buf);
                lcdBufferSetLine(1, "Press to save");
            } else {
                const menu_item_t* cur = menu.stack[menu.depth - 1];
                unsigned char cnt = menu_count(cur);
                if (cnt > 0) {
                    unsigned char sel = menu.selection[menu.depth - 1];
                    /* wrap using modulo arithmetic */
                    sel = (unsigned char)(sel + delta);
                    /* Ensure 0..cnt-1 */
                    sel %= cnt;
                    menu.selection[menu.depth - 1] = sel;
                    menu_draw();
                }
            }
        }
    }

    unsigned char btn = encoder_button_state();
    if (btn && !menu.last_button) {
        /* Button pressed event */
        if (!menu.active) {
            menu_open();
        } else {
            /* Act on selected item */
            const menu_item_t* cur = menu.stack[menu.depth - 1];
            unsigned char sel = menu.selection[menu.depth - 1];
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
                /* Leaf: handle editing for various menus */
                if (menu.stack[menu.depth - 1] == gps_menu) {
                    switch (sel) {
                        case 0: /* GPS Baud */
                            menu.editing = EDIT_GPS_BAUD;
                            menu.edit_value = baud_value_to_index(system_config.gps_baud);
                            break;
                        case 1: /* GPS Stop bits */
                            menu.editing = EDIT_STOPBITS;
                            menu.edit_value = (unsigned char)system_config.gps_stop_bits;
                            break;
                        case 2: /* GPS Parity */
                            menu.editing = EDIT_PARITY;
                            menu.edit_value = (unsigned char)system_config.gps_parity;
                            break;
                        case 3: /* GPS Protocol */
                            menu.editing = EDIT_GPS_PROTOCOL;
                            menu.edit_value = (unsigned char)system_config.gps_protocol;
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
                } else if (menu.stack[menu.depth - 1] == timezone_menu) {
                    switch (sel) {
                        case 0: /* UTC */
                            system_config.tz_mode = TZ_MODE_UTC;
                            config_save(&system_config);
                            menu_show_message("TZ: UTC", 800);
                            break;
                        case 1: /* Local (has submenu, shouldn't reach here) */
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
                } else if (menu.stack[menu.depth - 1] == offset_menu) {
                    /* User selected a specific timezone offset */
                    /* Calculate the offset from the menu index (0..105) */
                    /* Menu has: -12:00 through +14:00 in 15-minute steps, then Back */
                    if (sel < TZ_OFFSET_STEPS) {
                        int16_t offset_min = (int16_t)sel * 15 - 720;
                        system_config.tz_mode = TZ_MODE_LOCAL;
                        system_config.tz_offset_min = offset_min;
                        config_save(&system_config);
                        char buf[21];
                        char ofs[8];
                        tz_offset_to_string(offset_min, ofs);
                        memset(buf, ' ', sizeof(buf));
                        memcpy(buf, "TZ: ", 4);
                        memcpy(&buf[4], ofs, strlen(ofs));
                        buf[20] = '\0';
                        menu_show_message(buf, 800);
                    }
                } else if (menu.stack[menu.depth - 1] == faults_menu) {
                    /* User selected a fault item or Clear All */
                    if (sel < 5) {
                        /* Display fault details */
                        FaultRecord_t fault;
                        if (faultsGet(sel, &fault)) {
                            /* Show fault message on LCD (up to 4 lines of 20 chars each) */
                            lcdBufferClear();
                            char buf[21];
                            /* Line 1: Fault number and severity */
                            memset(buf, ' ', sizeof(buf));
                            buf[20] = '\0';
                            snprintf(buf, 21, "Fault %d - ", sel + 1);
                            switch (fault.severity) {
                                case FAULT_SEVERITY_INFO: strcat(buf, "INFO"); break;
                                case FAULT_SEVERITY_WARNING: strcat(buf, "WARN"); break;
                                case FAULT_SEVERITY_ERROR: strcat(buf, "ERR"); break;
                                case FAULT_SEVERITY_CRITICAL: strcat(buf, "CRIT"); break;
                            }
                            lcdBufferSetLine(0, buf);
                            /* Lines 2-3: Message (wrap if needed) */
                            memset(buf, ' ', sizeof(buf));
                            buf[20] = '\0';
                            size_t msg_len = strlen(fault.message);
                            if (msg_len <= 20) {
                                memcpy(buf, fault.message, msg_len);
                                lcdBufferSetLine(1, buf);
                            } else {
                                memcpy(buf, fault.message, 20);
                                lcdBufferSetLine(1, buf);
                                memset(buf, ' ', sizeof(buf));
                                size_t remaining = msg_len - 20;
                                if (remaining > 20) remaining = 20;
                                memcpy(buf, &fault.message[20], remaining);
                                lcdBufferSetLine(2, buf);
                            }
                            lcdBufferUpdate();
                            /* Display remains until user presses button to return to menu */
                        } else {
                            menu_show_message("No fault at slot", 2000);
                        }
                    } else if (sel == 5) {
                        /* Clear All faults */
                        faultsClear();
                        menu_show_message("Faults cleared", 1000);
                    }
                } else if (menu.stack[menu.depth - 1] == serial_menu) {
                    switch (sel) {
                        case 0: /* External Baud Rate */
                            menu.editing = EDIT_EXT_BAUD;
                            menu.edit_value = baud_value_to_index(system_config.ext_baud);
                            break;
                        case 1: /* Stop bits */
                            menu.editing = EDIT_STOPBITS;
                            menu.edit_value = (unsigned char)system_config.ext_stop_bits;
                            break;
                        case 2: /* External Parity */
                            menu.editing = EDIT_EXT_PARITY;
                            menu.edit_value = (unsigned char)system_config.ext_parity;
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
                        const char* text;
                        switch (menu.editing) {
                            case EDIT_VREF:
                                text = menu.edit_value == 0 ? "Internal" : "External";
                                memcpy(buf, "VRef:", 5);
                                memcpy(&buf[6], text, strlen(text));
                                break;
                            case EDIT_GPS_BAUD:
                                text = menu_get_text(baud_menu, menu.edit_value);
                                memcpy(buf, "GPS Baud:", 9);
                                memcpy(&buf[10], text, strlen(text));
                                break;
                            case EDIT_STOPBITS:
                                text = menu_get_text(stopbits_menu, menu.edit_value);
                                memcpy(buf, "GPS Stop:", 9);
                                memcpy(&buf[10], text, strlen(text));
                                break;
                            case EDIT_PARITY:
                                text = menu_get_text(parity_menu, menu.edit_value);
                                memcpy(buf, "GPS Parity:", 11);
                                memcpy(&buf[12], text, strlen(text));
                                break;
                            case EDIT_GPS_PROTOCOL:
                                text = menu_get_text(gps_protocol_menu, menu.edit_value);
                                memcpy(buf, "GPS Proto:", 10);
                                memcpy(&buf[11], text, strlen(text));
                                break;
                            case EDIT_EXT_BAUD:
                                text = menu_get_text(baud_menu, menu.edit_value);
                                memcpy(buf, "Ext Baud:", 9);
                                memcpy(&buf[10], text, strlen(text));
                                break;
                            case EDIT_EXT_PARITY:
                                text = menu_get_text(parity_menu, menu.edit_value);
                                memcpy(buf, "Ext Parity:", 11);
                                memcpy(&buf[12], text, strlen(text));
                                break;
                            case EDIT_TZ_MODE:
                                text = menu_get_text(timezone_menu, menu.edit_value);
                                memcpy(buf, "TZ Mode:", 8);
                                memcpy(&buf[9], text, strlen(text));
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
                system_config.vref_source = (vref_source_t)menu.edit_value;
                break;
            case EDIT_GPS_BAUD:
                system_config.gps_baud = baud_str_to_value(menu_get_text(baud_menu, menu.edit_value));
                break;
            case EDIT_STOPBITS:
                system_config.gps_stop_bits = (stopbits_t)menu.edit_value;
                break;
            case EDIT_PARITY:
                system_config.gps_parity = (parity_t)menu.edit_value;
                break;
            case EDIT_GPS_PROTOCOL:
                system_config.gps_protocol = (gps_protocol_t)menu.edit_value;
                gps_set_protocol((gps_protocol_t)menu.edit_value);
                break;
            case EDIT_EXT_BAUD:
                system_config.ext_baud = baud_str_to_value(menu_get_text(baud_menu, menu.edit_value));
                serial_reconfigure(); // Apply new settings immediately
                break;
            case EDIT_EXT_PARITY:
                system_config.ext_parity = (parity_t)menu.edit_value;
                serial_reconfigure(); // Apply new settings immediately
                break;
            case EDIT_TZ_MODE:
                system_config.tz_mode = (tz_mode_t)menu.edit_value;
                break;
            case EDIT_TZ_OFFSET:
                /* map index back to minutes */
                system_config.tz_mode = TZ_MODE_LOCAL;
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
