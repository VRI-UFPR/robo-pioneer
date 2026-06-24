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
#include "pioneer_p3dx.h"

// ============================================================================
//  Main
// ============================================================================

int main () {
    // Start
    pioneer_connect("/dev/pioneer", 2);
    // pioneer_disable_sonars();
    // pioneer_enable_sonars();
    // pioneer_enable_motors();

    // Movimentando para frente
    printf("# Movimentando para frente\n");
    for (int i=0; i<10; i++) {
        pioneer_vel2(5, 5);
        usleep(500000); // 500ms
    }

    // Girando para os lados
    printf("# Girando para os lados\n");
    pioneer_vel(0);
    pioneer_rotvel(15);
    sleep(2);
    pioneer_rotvel(-15);
    sleep(2);
    
    // Fim
    printf("fim\n");
    pioneer_close();
    return 0;
}