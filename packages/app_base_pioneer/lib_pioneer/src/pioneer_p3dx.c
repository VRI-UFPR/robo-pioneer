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

#include "pioneer_p3dx.h"
#include "pioneer_p3dx_pvt.h"

pioneer_t g_pioneer;
int g_debug_level = 0;

// ============================================================================
//  Pioneer Basic
// ============================================================================

int pioneer_read_block(pioneer_t* pioneer, const uint8_t count, uint8_t* buffer) {
    int rest = count;
    int wrte = 0;
    int tried = 0;
    do {
        const int size = read(pioneer->tty.fd, &buffer[wrte], rest);
        if ( size == 0 ) {
            tty_select(&pioneer->tty, 10);
            tried += 1;
            if ( tried > 3 ) {
                return 0;
            }
        } else {
            rest -= size;
            wrte += size;
        }
    } while ( rest > 0 );
    return wrte;
}

int pioneer_read_package(pioneer_t* pioneer) {
    // read data from serial port
    int pack_size = 0;
    uint8_t buffer[PACK_MAX_SIZE];
    int tried = 0;

    int state = 0;
    for(int i=0; i<3;) {
        uint8_t c = 0;
        const int size = read(pioneer->tty.fd, &c, 1);
        if ( size == 0 ) {
            tty_select(&pioneer->tty, 25);
            tried += 1;
            if ( tried > 20 ) {
                fprintf(stderr, "Timeout - nenhum dado recebido\n");
                exit(1);
            }
            continue;
        } else {
            tried = 0;
        }

        // waiting for 0xFA
        if ( state == 0 ) {
            if ( c == 0xFA ) {
                buffer[0] = c;
                state = 1;
            }
            i += 1;

        // waiting for 0xFB
        } else if ( state == 1 ) {
            if ( c == 0xFB ) {
                buffer[1] = c;
                state = 2;
            } else {
                state = 0;
            }

        // read the size of package
        } else if ( state == 2 ) {
            buffer[2] = c;
            const uint8_t count = c;
            const uint8_t size = pioneer_read_block(pioneer, count, &buffer[3]);
            if ( size == 0 ) {
                // printf("error 1\r\n"); - arrumar este erro
                return 0;
            }
            pack_size = 3 + size;
            break;
        }
    }

    // Success
    if ( pack_size > 3 ) {
        uint8_t cmd = buffer[3];
        if ( cmd == 0x32 || cmd == 0x33 ) {
            pioneer->pos_x = *( (int16_t*) &buffer[4] );
            pioneer->pos_y = *( (int16_t*) &buffer[6] );
            uint16_t u16_pos_th = *( (int16_t*) &buffer[8] );
            pioneer->pos_th = (((float) u16_pos_th) * 360.0 ) / 4096.0;
        }
    }

    if ( g_debug_level >= 2 ) {
        fprintf(stderr, "recv: ");
        for (int i=0; i<pack_size; i++) {
            fprintf(stderr, "%02X ", buffer[i]);
        }
        fprintf(stderr, "\r\n");
        fprintf(stderr, "- x: %d, y: %d \r\n", pioneer->pos_x, pioneer->pos_y);
    }

    return pack_size;
}


int pioneer_exec(pioneer_t* pioneer, uint8_t command) {
    pack_init(pioneer->pack_send, command);
    pack_finish(pioneer->pack_send);
    tty_send_pack(&pioneer->tty, pioneer->pack_send);
    return OK;
}

int pioneer_exec_with_answer(pioneer_t* pioneer, uint8_t command) {
    pack_init(pioneer->pack_send, command);
    pack_finish(pioneer->pack_send);
    tty_send_pack(&pioneer->tty, pioneer->pack_send);
    pioneer_read_package(pioneer);
    return OK;
}

int pioneer_exec_va(pioneer_t* pioneer, uint8_t command, char* format, ...) {
    // init package
    va_list list;
    va_start(list, format);
    pack_init(pioneer->pack_send, command);

    // put the variable
    for (int i=0; i<5; i++) {
        const char c = format[i];
        if ( c == '\0' || c == '\n' ) {
            break;
        }
        if ( c == 'i' ) {
            int16_t val = (int16_t) va_arg(list, int32_t);
            pack_put_i16(pioneer->pack_send, val);
        } else if ( c == 'u' ) {
            uint16_t val = (uint16_t) va_arg(list, int32_t);
            pack_put_u16(pioneer->pack_send, val);
        }
    }

    va_end(list);

    // put checksum,  send and receive
    pack_finish(pioneer->pack_send);
    tty_send_pack(&pioneer->tty, pioneer->pack_send);

    // success
    return OK;
}

// ============================================================================
//  Pioneer Constructor
// ============================================================================

/**
 * Inicializa o objeto da classe pioneer. Apenas zera as variaveis e abre a
 * conexao com o dispositivo serial /dev/ttyUSB0 com a velocidade 9600
 */

__attribute__((constructor))
int pioneer_init() {
    g_pioneer.pack_send = (pack_t*) &g_pioneer.buf_send[0];
    g_pioneer.pos_x = 0;
    g_pioneer.pos_y = 0; 
    g_pioneer.pos_th = 0;
    return OK;
}

// ============================================================================
//  Pioneer Commands
// ============================================================================

/**
 * Envia comandos para se conectar com o robo
 * 
 * @brief
 *          Ass     | Count | Command | Checksum
 * -----------------------------------------------
 * sent1: 0xFA 0xFB | 0x03  | 0x00    | 0x00 0x00
 * recv1: 0xFA 0xFB | 0x03  | 0x00    | 0x00 0x00
 * 
 * sent2: 0xFA 0xFB | 0x03  | 0x01    | 0x00 0x01
 * recv2: 0xFA 0xFB | 0x03  | 0x01    | 0x00 0x01
 * 
 * sent3: 0xFA 0xFB | 0x03  | 0x02    | 0x00 0x02
 * recv3: 0xFA 0xFB | 0x03  | 0x02    | 0x00 0x02
 * 
 * sent4: 0xFA 0xFB | 0x03  | 0x01    | 0x00 0x01
 * recv4:
 *
 * @return error code
 */

int pioneer_connect(const char* device_path, int debug_level) {
    g_debug_level = debug_level;

    assert( tty_init(&g_pioneer.tty, device_path, B9600) == OK );

    // Sync0,1,2
    assert( pioneer_exec_with_answer(&g_pioneer, 0x00) == OK );
    assert( pioneer_exec_with_answer(&g_pioneer, 0x01) == OK );
    assert( pioneer_exec_with_answer(&g_pioneer, 0x02) == OK );

    // Open server
    assert( pioneer_exec_with_answer(&g_pioneer, 0x01) == OK );
    assert( pioneer_enable_motors() == OK );

    // Success
    return OK;
}

/**
 * Fecha conexao com o robo
 * 
 * @brief
 *          Ass    | Count | Command | Checksum
 * -----------------------------------------------
 * sent: 0xFA 0xFB | 0x03  | 0x02    | 0x00 0x02
 *
 * @return error code
 */
int pioneer_close() {
    return pioneer_exec(&g_pioneer, 0x02);
}

/**
 * Envia um pulse para robo para dizer que o algoritmo 
 * esta vivo
 * 
 * @brief
 *          Ass    | Count | Command | Checksum
 * -----------------------------------------------
 * sent: 0xFA 0xFB | 0x03  | 0x00    | 0x00 0x00
 *
 * @return error code
 */
int pioneer_pulse() {
    return pioneer_exec(&g_pioneer, 0x00);
}

/**
 * Envia um pulse para robo para dizer que o algoritmo 
 * esta vivo
 * 
 * @brief
 *          Ass    | Count | Command | ArgType1      | ArgValue1 | Checksum
 * -------------------------------------------------------------------------
 * sent: 0xFA 0xFB | 0x06  | 28      | 0x1B (uint16) | 0x01 0x00 | 0x00 0x00
 *
 * @return error code
 */
int pioneer_enable_sonars() {
    return pioneer_exec_va(&g_pioneer, 28, "u", 1);
}

/**
 * Envia um pulse para robo para dizer que o algoritmo 
 * esta vivo
 * 
 * @brief
 *          Ass    | Count | Command | ArgType1      | ArgValue1 | Checksum
 * -------------------------------------------------------------------------
 * sent: 0xFA 0xFB | 0x06  | 28      | 0x1B (uint16) | 0x00 0x00 | 0x00 0x00
 *
 * @return error code
 */
int pioneer_disable_sonars() {
    return pioneer_exec_va(&g_pioneer, 28, "u", 0);
}

/**
 * Envia um pulse para robo para dizer que o algoritmo 
 * esta vivo
 * 
 * @brief
 *          Ass    | Count | Command | ArgType1      | ArgValue1 | Checksum
 * -------------------------------------------------------------------------
 * sent: 0xFA 0xFB | 0x06  | 4       | 0x1B (uint16) | 0x01 0x00 | 0x00 0x00
 *
 * @return error code
 */
int pioneer_enable_motors() {
    return pioneer_exec_va(&g_pioneer, 4, "u", 1);
}

/**
 * Envia um pulse para robo para dizer que o algoritmo 
 * esta vivo
 * 
 * @brief
 *          Ass    | Count | Command | ArgType1      | ArgValue1 | Checksum
 * -------------------------------------------------------------------------
 * sent: 0xFA 0xFB | 0x06  | 4       | 0x1B (uint16) | 0x00 0x00 | 0x00 0x00
 *
 * @return error code
 */
int pioneer_disable_motors() {
    return pioneer_exec_va(&g_pioneer, 4, "u", 0);
}

/**
 * Envia um pulse para robo para dizer que o algoritmo 
 * esta vivo
 * 
 * @brief
 *          Ass    | Count | Command | ArgType1     | ArgValue1     | Checksum
 * -------------------------------------------------------------------------
 * sent: 0xFA 0xFB | 0x06  | 11      | 0x3B (int16) | vel[0] vel[1] | 0x?? 0x??
 *
 * @return error code
 */
int pioneer_vel(int16_t vel) {
    return pioneer_exec_va(&g_pioneer, 11, "i", vel);
}

/**
 * Envia um pulse para robo para dizer que o algoritmo 
 * esta vivo
 * 
 * @brief
 *          Ass    | Count | Command | ArgType1     | ArgValue1     | Checksum
 * -------------------------------------------------------------------------
 * sent: 0xFA 0xFB | 0x06  | 32      | 0x3B (int16) | vel1 vel2     | 0x?? 0x??
 *
 * @return error code
 */
int pioneer_vel2(int8_t vel1, int8_t vel2) {
    uint16_t vel;
    vel = vel1;
    vel += ((uint16_t) vel2) << 8;
    return pioneer_exec_va(&g_pioneer, 32, "i", vel);
}

/**
 * Envia um pulse para robo para dizer que o algoritmo 
 * esta vivo
 * 
 * @brief
 *          Ass    | Count | Command | ArgType1     | ArgValue1               | Checksum
 * ---------------------------------------------------------------------------------------
 * sent: 0xFA 0xFB | 0x06  | 21      | 0x3B (int16) | rotvel[0] rotval[1]     | 0x?? 0x??
 *
 * @return error code
 */
int pioneer_rotvel(int16_t rotvel) {
    return pioneer_exec_va(&g_pioneer, 21, "i", rotvel);
}

int pioneer_read(int* pos_x, int* pos_y, float* pos_th) {
    if ( pioneer_read_package(&g_pioneer) > 0 ) {
        *pos_x = g_pioneer.pos_x;
        *pos_y = g_pioneer.pos_y;
        *pos_th = g_pioneer.pos_th;
    }
}