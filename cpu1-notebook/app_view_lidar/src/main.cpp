/* BSD 2-Clause License
 * 
 * Copyright (c) 2024, Visao Robotica e Imagem (VRI)
 *  - Felipe Bombardelli <felipebombardelli@gmail.com>
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// ============================================================================
//  Header
// ============================================================================

#include <ufr.h>
#include <unistd.h>
#include <math.h>

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

#define ROWS  480
#define COLS  640

// ============================================================================
//  Test
// ============================================================================

int main() {
    // Inicializa as variaveis
    const int32_t cy = ROWS / 2;
    const int32_t cx = COLS / 2;
    Mat image(480, 640, CV_8UC1);
    link_t* scan = ufr_subscriber_env("UFR_SCAN");

    // loop principal
    float angle_min, angle_max, angle_increment;
    while( ufr_loop() ) {
        if ( ufr_recv(scan) == false ) {
            break;
        }

        // Faz a leitura das distancias
        float valors[1024];
        ufr_get(scan, "%f %f %f %- %- %- %-", &angle_min, &angle_max, &angle_increment);
        const int nitems = ufr_get_af32(scan, valors, 1024);

        printf("%d %f %f %f\n", nitems, angle_min, angle_max, angle_increment);

        // 
        image.setTo(Scalar(0));
        float angle = angle_min;
        for (int i=0; i<nitems; i++, angle+=angle_increment) {
            // Le a distancia e verifica se é um valor valido
            const float distance = valors[i];
            if ( isnan(distance) || !isfinite(distance) ) {
                continue;
            }

            // Calcula a posição do pixel
            // printf("%f\n", distance);
            const int32_t dx = round(5.0 * distance * cos(angle));
            const int32_t dy = round(5.0 * distance * -sin(angle));
            const uint32_t py = cy + dy;
            const uint32_t px = cy + dy;

            // Verifica se o pixel está dentro da imagem
            if ( py < ROWS && px < COLS ) {
                image.at<uint8_t>(cy+dy,cx+dx) = 255;
            }
        }

        // Show the image
        imshow("lidar", image);
        waitKey(1);
    }

    // Fim
    printf("Fim\n");
    ufr_close(scan);
    return 0;
}
