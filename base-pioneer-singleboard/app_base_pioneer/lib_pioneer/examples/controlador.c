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

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ufr.h>
#include <signal.h>

// ============================================================================
//  Main Pioneer
// ============================================================================


int main(int argc, char *argv[]) {
    link_t timer = ufr_subscriber("@new posix:timer @time 250ms");
    // link_t odom = ufr_publisher("@new ros_humble:topic @msg twist @topic /odom");

    while( 1 ) {
        if ( ufr_recv_async(&timer) == UFR_OK ) {
            printf("opa\n");
        }

        // pioneer_pulse();
        /*int pos_x=1, pos_y=1;
        float pos_th=2;
        // pioneer_read(&pos_x, &pos_y, &pos_th);
        ufr_put(&odom, "iif\n", pos_x, pos_y, pos_th);
        sleep(1);*/
    }

    // End
    // pioneer_close();
    return 0;
}