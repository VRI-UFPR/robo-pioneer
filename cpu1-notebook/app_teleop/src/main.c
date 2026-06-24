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
//  Cabeçalho
// ============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <ufr.h>

struct termios orig_termios;

#ifndef ROBOT_TOPIC_CMD_VEL
#define ROBOT_TOPIC_CMD_VEL "@new mqtt @coder msgpack @host 177.153.62.174 @topic /pioneer/cmd_vel"
#endif

// ============================================================================
//  Funções
// ============================================================================

// Restore terminal to its original state on exit
void reset_terminal_mode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
}

// Disable line buffering and echoing
void set_conio_terminal_mode() {
    struct termios new_termios;
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(reset_terminal_mode);
    new_termios = orig_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
}

// ============================================================================
//  Main
// ============================================================================

int main() {
    // Inicializa o teclado no modo assincrono
    set_conio_terminal_mode();
    struct pollfd pfd = { .fd = STDIN_FILENO, .events = POLLIN };
    char c;
    printf("Press keys (Press 'q' to quit)...\n");

    // Inicializa a comunicação com o roomba
    link_t* cmdvel = ufr_publisher_env("UFR_CMDVEL");

    // Loop principal
    while ( ufr_loop() ) {
        // Poll for 100ms. Returns > 0 if a key was pressed.
        int ret = poll(&pfd, 1, 100);

        if (ret > 0 && (pfd.revents & POLLIN)) {
            if (read(STDIN_FILENO, &c, 1) > 0) {
                // Caso apertado q: sai do programa
                if (c == 'q') {
                    break;
                }

                // Caso apertado w, s, a, d, altera a velocidade do roomba 
                if ( c == 'w' ) {
                    ufr_put(cmdvel, "%f %f\n", 1.0, 0.0);

                } else if ( c == 's' ) {
                    ufr_put(cmdvel, "%f %f\n", -1.0, 0.0);

                } else if ( c == 'a' ) {
                    ufr_put(cmdvel, "%f %f\n", 0.0, 10.0);

                } else if ( c == 'd' ) {
                    ufr_put(cmdvel, "%f %f\n", 0.0, -10.0);

                // Caso apertado espaço, pára o roomba
                } else if ( c == ' ' ) {
                    ufr_put(cmdvel, "%f %f\n", 0.0, 0.0);

                // Nenhum dos casos acimas, simplesmente mostra a tecla
                } else {
                    printf("Key pressed: %c\n", c);
                    fflush(stdout);
                }
            }
        }
    }

    // Pára o movimento e fecha a comunicação com o roomba
    ufr_put(cmdvel, "%f %f\n", 0.0, 0.0);
    ufr_close(cmdvel);
    return 0;
}