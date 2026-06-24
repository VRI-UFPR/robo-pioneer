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

#include <assert.h>
#include <errno.h>
#include <fcntl.h> 
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "pioneer_p3dx_pvt.h"

// ============================================================================
//  Package Functions
// ============================================================================

void pack_init(pack_t* pack, uint8_t command) {
    pack->header[0] = 0xFA;
    pack->header[1] = 0xFB;
    pack->count = 0;
    pack->command = command;
}

void pack_put_u16(pack_t* pack, uint16_t value) {
    if ( pack->count + sizeof(value) >= PACK_MAX_SIZE ) {
        return;
    }

    // put the integer signature
    pack->argdata[ pack->count ] = TYPE_U16;
    pack->count += 1;

    // put the integer value
    int16_t* buf = (int16_t*) &pack->argdata[ pack->count ];
    *buf = value;
    pack->count += sizeof(value);
}

void pack_put_i16(pack_t* pack, int16_t value) {
    if ( pack->count + sizeof(value) >= PACK_MAX_SIZE ) {
        return;
    }

    // put the integer signature
    pack->argdata[ pack->count ] = TYPE_I16;
    pack->count += 1;

    // put the integer value
    int16_t* buf = (int16_t*) &pack->argdata[ pack->count ];
    *buf = value;
    pack->count += sizeof(value);
}

void pack_finish(pack_t* pack) {
    // Update package counter with the command byte
    pack->count += 1;

    // calculate the checksum
    int i = 3;
    int c = 0;
    uint8_t* buffer = (uint8_t*) pack;
    uint8_t n = pack->count;
    while (n > 1) {
        c += (buffer[i]<<8) | buffer[i+1];
        c = c & 0xffff;
        n -= 2;
        i += 2;
    }
    if (n > 0) {
        c = c ^ (int)(buffer[i]);
    }

    // put the checksum in the package
    const uint16_t checksum = c & 0xFFFF;
    const uint8_t pos = offsetof(pack_t,command) + pack->count;
    // printf("pos: %d %x %x\n", pos, checksum, (checksum >> 8) & 0xFF  );
    
    buffer[pos] = (uint8_t) (checksum >> 8) & 0xFF;
    buffer[pos+1]   = (uint8_t) checksum & 0xFF;

    // update package counter (+2 bytes of checksum)
    pack->count += 2;
}

void pack_print(pack_t* pack) {
    printf("header: 0x%X 0x%X\n", pack->header[0], pack->header[1]);
    printf("count: %d\n", pack->count);
    printf("command: 0x%X\n", pack->command);
    printf("argdata: [");
    for (int i=1; i<pack->count; i++) {
        printf("%02x ", pack->argdata[i]);
    }
    printf("]\n");
    uint16_t* checksum = (uint16_t*) &pack->argdata[ pack->count-2 ];
    printf("checksum: 0x%X\n\n", *checksum);
}
