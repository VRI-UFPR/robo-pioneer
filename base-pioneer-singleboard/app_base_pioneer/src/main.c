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
#include <math.h>

#include "pioneer_p3dx.h"


// ============================================================================
//  Main Pioneer
// ============================================================================

int main(int argc, char *argv[]) {
    link_t* timer = ufr_subscriber("@new timer @time 200ms");
    link_t* vel_cmd = ufr_subscriber_env("UFR_CMDVEL");
    link_t* odom = ufr_publisher("UFR_ODOM");

    const char* serial_port = (argc >= 2) ? argv[1] : "/dev/ttyUSB0";
    pioneer_connect(serial_port, 0);
    pioneer_disable_sonars();
    // -- pioneer_enable_motors();

    // Main loop
    int count = 0;   
    int16_t i16_vel = 0;
    int16_t i16_rotvel = 0;
    float vel=0, rotvel=0;
    while( ufr_loop_ok() ) {

        if ( ufr_recv_async(vel_cmd) ) {
            ufr_get(vel_cmd, "%f %f", &vel, &rotvel);
            i16_vel = (int16_t) (vel * 100.0);
            i16_rotvel = (int16_t) rotvel;
            printf("%f %f\n", vel, rotvel);

            if ( i16_vel > 150 ) {
                i16_vel = 150;
            } else if ( i16_vel < -150 ) {
                i16_vel = -150;
            }

            if ( i16_rotvel > 10 ) {
                i16_rotvel = 10;
            } else if ( i16_rotvel < -10 ) {
                i16_rotvel = -10;
            }

            pioneer_vel( i16_vel );
            pioneer_rotvel( i16_rotvel );
	        printf("[LOG]: %d %d\n", i16_vel, i16_rotvel);
            count = 0;
        }

        if ( ufr_recv_async(timer) == UFR_OK ) {
            if ( count >= 3 ) {
                pioneer_vel(i16_vel);
                pioneer_rotvel(i16_rotvel);
                printf("[LOG]: %d %d\n", i16_vel, i16_rotvel);
                count = 0;
                printf("[LOG]: Timeout\n");
            }
            count += 1;
        }

        // publishes the odometry
        int pos_x, pos_y;
        float pos_th;
        pioneer_read(&pos_x, &pos_y, &pos_th);
        ufr_put(odom, "%d %d %f %f %f\n", pos_x, pos_y, pos_th*M_PI/180.0, vel, rotvel);

        // wait 50ms
        usleep(50000);
    }

    // End
    pioneer_close();
    return 0;
}