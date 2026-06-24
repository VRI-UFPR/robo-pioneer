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

#include <stdint.h>

#define OK      0
#define ERROR   1

// ============================================================================
//  Public Functions
// ============================================================================

// Connect and disconnect the Pioneer P3DX robot
int pioneer_connect(const char* device_path, int debug_level);
int pioneer_close();
int pioneer_pulse();

// Disable and Enable Sonars
int pioneer_disable_sonars();
int pioneer_enable_sonars();

// Disable and Enable Motors
int pioneer_disable_motors();
int pioneer_enable_motors();

// Set speed
int pioneer_vel(int16_t vel);
int pioneer_vel2(int8_t vel1, int8_t vel2);
int pioneer_rotvel(int16_t rotvel);

// Read data from Pioneer
int pioneer_read(int* pos_x, int* pos_y, float* pos_th);