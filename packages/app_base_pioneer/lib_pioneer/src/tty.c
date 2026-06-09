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
//  TTY Functions
// ============================================================================

int tty_init(tty_t* tty, const char* device_path, int baud) {
    printf("Connecting on %s\n", device_path);

    int tty_fd = open(device_path, O_RDWR | O_NOCTTY); // "/dev/ttyUSB0"

    // ArSerialConnection_LIN.cpp
    struct termios tio;

    if (tcgetattr(tty_fd, &tio) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        exit(1);
    }

    /* turn off echo, canonical mode, extended processing, signals */
    tio.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);

    /* turn off break sig, cr->nl, parity off, 8 bit strip, flow control */
    tio.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

    /* clear size, turn off parity bit */
    tio.c_cflag &= ~(CSIZE | PARENB);

    /* set size to 8 bits */
    tio.c_cflag |= CS8;

    /* turn output processing off */
    tio.c_oflag &= ~(OPOST);

    /* Set time and bytes to read at once */
    tio.c_cc[VTIME] = 0;
    tio.c_cc[VMIN] = 0;

    // set baut 115200
    if (cfsetospeed(&tio, baud)) 
    {
        printf("error 1\n");
        exit(1);
    }
       
    if (cfsetispeed(&tio, baud)) 
    {
        printf("error 2\n");
        exit(1);
    }

    // flush
    if (tcflush(tty_fd,TCIFLUSH) == -1) {
        printf("error 3\n");
        exit(1);
    }

    if (tcsetattr(tty_fd,TCSAFLUSH,&tio) == -1) {
        printf("error 4\n");
        exit(1);
    }

    tty->fd = tty_fd;
    return OK;
}

void tty_send_pack(tty_t* tty, pack_t* pack) {
    const uint8_t* pack_ptr = (uint8_t*) pack;
    const size_t pack_size = offsetof(pack_t,command) + pack->count;
    const int sent =  write( tty->fd, pack_ptr, pack_size);
    if ( sent != pack_size ) {
        fprintf(stderr, "erro ao enviar\n");
    }

    // Debug
    if ( g_debug_level >= 2 ) {
        fprintf(stderr, "sent: ");
        for ( int i=0; i<pack_size; i++) {
            fprintf(stderr, "%02x ", pack_ptr[i]);
        }
        fprintf(stderr, "\r\n");
    }
}

int tty_select(tty_t* tty, int timeout_ms) {
    // configure the time of timeout
    struct timeval tp;
    tp.tv_sec = (timeout_ms) / 1000;	
	tp.tv_usec = (timeout_ms % 1000) * 1000;
    fd_set fdset;
    FD_ZERO(&fdset);
    FD_SET(tty->fd,&fdset);

    // -1: error
    return select(tty->fd, &fdset, NULL, NULL, &tp);
}

int tty_read(tty_t* tty, int timeout_ms, uint8_t* buffer, int maxsize) {
    // -1: error, 0: timeout: out
    const int error = tty_select(tty, timeout_ms);
    if (error < 0) {
        printf("timeout %d\n", error);
        return 0;
    }

    // read and return size of package
    const int size = read(tty->fd, buffer, maxsize);
    return size;
}