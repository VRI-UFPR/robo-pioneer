/**
 * Pioneer-P3DX communication library
 *  Copyright (C) 2024  Visao Robotica e Imagem (VRI)
 *   - Felipe Bombardelli <felipebombardelli@gmail.com>
 * 
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 * * */

// ============================================================================
//  Header
// ============================================================================

#include "pioneer_p3dx.h"

#define TYPE_U16  0x1b
#define TYPE_I16  0x3b
#define TYPE_STR  0x2b

#define PACK_MAX_SIZE 256

extern int g_debug_level;

// ============================================================================
//  Strutures
// ============================================================================

typedef struct {
    uint8_t header[2];
    uint8_t count;
    uint8_t command;
    uint8_t argdata[];
} pack_t;

typedef struct {
    int state;
    int fd;
} tty_t;

typedef struct {
    uint8_t buf_send[PACK_MAX_SIZE];
    pack_t* pack_send;
    
    tty_t tty;
    int16_t pos_x;
    int16_t pos_y;
    float pos_th;
} pioneer_t;

// ============================================================================
//  Package Functions
// ============================================================================

void pack_init(pack_t* pack, uint8_t command);
void pack_put_u16(pack_t* pack, uint16_t value);
void pack_put_i16(pack_t* pack, int16_t value);
void pack_finish(pack_t* pack);
void pack_print(pack_t* pack);

// ============================================================================
//  TTY Functions
// ============================================================================

int tty_init(tty_t* tty, const char* device_path, int baud);
void tty_send_pack(tty_t* tty, pack_t* pack);
int tty_select(tty_t* tty, int timeout_ms);
int tty_read(tty_t* tty, int timeout_ms, uint8_t* buffer, int maxsize);