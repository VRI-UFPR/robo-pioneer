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

// ============================================================================
//  Testing Code
// ============================================================================

#include "../src/pack.c"

// ============================================================================
//  Tests
// ============================================================================

void test1() {
    // Create a buffer of 256 bytes as pack_t
    uint8_t pack_buffer[256];
    pack_t* pack = (pack_t*) pack_buffer;

    // initialize the package
    pack_init(pack, 0x01);
    assert( pack->header[0] == 0xFA );
    assert( pack->header[1] == 0xFB );
    assert( pack->command == 0x01 );
    assert( pack->count == 0 );
    
    pack_finish(pack);
    assert( pack->count == 3 );
    assert( pack->argdata[0] == 0x00 );
    assert( pack->argdata[1] == 0x01 );
}

void test2() {
    // Create a buffer of 256 bytes as pack_t
    uint8_t pack_buffer[256];
    pack_t* pack = (pack_t*) pack_buffer;

    // initialize the package
    pack_init(pack, 28);
    assert( pack->command == 28 );

    // put unsigned int
    pack_put_u16(pack, 0x1234);
    assert( pack->count == 3 );
    assert( pack->argdata[0] == TYPE_U16 );
    assert( pack->argdata[1] == 0x34 );
    assert( pack->argdata[2] == 0x12 );

    pack_put_i16(pack, 0x1234);
    assert( pack->count == 6 );
    assert( pack->argdata[3] == TYPE_I16 );
    assert( pack->argdata[4] == 0x34 );
    assert( pack->argdata[5] == 0x12 );

    pack_finish(pack);
    assert( pack->count == 9 );
    // assert( pack->argdata[6] == 0x00 );
    // assert( pack->argdata[7] == 0x01 );
    pack_print(pack);
}

// ============================================================================
//  Main
// ============================================================================

int main() {
    test1();
    test2();
}