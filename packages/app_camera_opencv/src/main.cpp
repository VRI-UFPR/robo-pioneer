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

#include <stdio.h>
#include <stdlib.h>
#include <ufr.h>
#include <unistd.h>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#ifndef ROBOT_TOPIC_CAMERA_RGB
#define ROBOT_TOPIC_CAMERA_RGB "@new mqtt @coder msgpack @topic camera_rgb"
#endif

// ============================================================================
//  Main
// ============================================================================

int main() {
    link_t video = ufr_publisher(ROBOT_TOPIC_CAMERA_RGB);
    ufr_exit_if_error(&video);

    cv::VideoCapture cap(0);
    std::vector<uint8_t> buffer;
    while( ufr_loop_ok() ) {
        Mat frame;
        cap.read(frame);
        imencode(".jpg", frame, buffer);

        ufr_put(&video, "sii", ".jpg", frame.cols, frame.rows);
        ufr_put_raw(&video, &buffer[0], buffer.size());
        ufr_send(&video);
    }

    // fim
    ufr_close(&video);
    return 0;
}

