/*
 * Copyright (c) 2025, Dewayne L. Hafenstein.  All rights reserved.
 *
 * Simple hierarchical menu driven by the rotary encoder and displayed on the LCD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *  http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MENU_H
#define MENU_H

#include <stdint.h>

void menu_init(void);       // call once at startup 
void menu_process(void);    // call regularly from main loop (~10ms) 
void menu_open(void);       // open the menu 
void menu_close(void);      // close the menu

#endif // MENU_H
