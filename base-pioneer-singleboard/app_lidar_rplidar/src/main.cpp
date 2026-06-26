/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

// ============================================================================
//  HEADER
// ============================================================================

#include <string>
#include <cstring>
#include <limits>
#include <memory>
#include <chrono>
#include <ufr.h>

#include "rplidar.h"

#define M_PI 3.1415926535897932384626433832795
#define MAX_SCAN_COUNT  (360*8)

using namespace rp::standalone::rplidar;
using ResponseNodeArray = std::unique_ptr<rplidar_response_measurement_node_hq_t[]>;

RPlidarDriver* m_drv = NULL;
const float min_distance = 0.15f;
size_t m_angle_compensate_multiple;
bool angle_compensate_;
bool inverted_;
bool flip_x_axis_;
double angle_min;
double angle_max;
double max_distance = 8.0f;

link_t* g_pub = NULL;


// ============================================================================
//  Functions
// ============================================================================

constexpr double deg_2_rad(double x) {
  return x * M_PI / 180.0;
}

void rplidar_free() {
    if (nullptr == m_drv) {
        return;
    }

    RPlidarDriver::DisposeDriver(m_drv);
    m_drv = NULL;
}

void start_motor() {
    if (nullptr == m_drv) {
        return;
    }

    m_drv->startMotor();
    m_drv->startScan(0, 1);
}

void stop_motor() {
    if (nullptr == m_drv) {
        return;
    }

    m_drv->stop();
    m_drv->stopMotor();
}

static float getAngle(const rplidar_response_measurement_node_hq_t& node) {
    return node.angle_z_q14 * 90.f / 16384.f;
}

bool getRPLIDARDeviceInfo() {
    u_result op_result;
    rplidar_response_device_info_t devinfo;

    op_result = m_drv->getDeviceInfo(devinfo);
    if (IS_FAIL(op_result)) {
        if (op_result == RESULT_OPERATION_TIMEOUT) {
            // RCLCPP_ERROR(this->get_logger(), "Error, operation time out. RESULT_OPERATION_TIMEOUT!");
        } else {
            // RCLCPP_ERROR(this->get_logger(), "Error, unexpected error, code: '%x'", op_result);
        }
        return false;
    }

    // print out the device serial number, firmware and hardware version number..
    std::string serial_no{"RPLIDAR S/N: "};
    for (int pos = 0; pos < 16; ++pos) {
        char buff[3];
        snprintf(buff, sizeof(buff), "%02X", devinfo.serialnum[pos]);
        serial_no += buff;
    }

    printf("%s\n", serial_no.c_str());
    printf("Firmware Ver: %d.%02d\n", devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF);
    printf("Hardware Rev: %d\n", static_cast<int>(devinfo.hardware_version));

    // RCLCPP_INFO(this->get_logger(), "%s", serial_no.c_str());
    // RCLCPP_INFO(this->get_logger(), "Firmware Ver: %d.%02d", devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF);
    // RCLCPP_INFO(this->get_logger(), "Hardware Rev: %d", static_cast<int>(devinfo.hardware_version));
    return true;
}

bool checkRPLIDARHealth() {
    rplidar_response_device_health_t healthinfo;
    u_result op_result = m_drv->getHealth(healthinfo);

    if (IS_OK(op_result)) {
        // RCLCPP_INFO(this->get_logger(), "RPLidar health status : '%d'", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            // RCLCPP_ERROR( this->get_logger(),
            //    "Error, rplidar internal error detected. Please reboot the device to retry");
            return false;
        }
        return true;
    }
    // RCLCPP_ERROR(this->get_logger(), "Error, cannot retrieve rplidar health code: '%x'", op_result);
    return false;
}

bool set_scan_mode() {
    RplidarScanMode current_scan_mode;
    
    const u_result op_result = m_drv->startScan(
        false /* not force scan */, true /* use typical scan mode */, 0,
        &current_scan_mode);

    /* verify we set the scan mode */
    if (!IS_OK(op_result)) {
        // RCLCPP_ERROR(this->get_logger(), "Cannot start scan: '%08x'", op_result);
        return false;
    }

    // default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
    m_angle_compensate_multiple =
    static_cast<int>(1000 * 1000 / current_scan_mode.us_per_sample / 10.0 / 360.0);
    if (m_angle_compensate_multiple < 1) {
        m_angle_compensate_multiple = 1;
    }

    max_distance = current_scan_mode.max_distance;
    // RCLCPP_INFO( this->get_logger(),
    //    "current scan mode: %s, max_distance: %.1f m, Point number: %.1fK , angle_compensate: %d, flip_x_axis %d", current_scan_mode.scan_mode,
    //    current_scan_mode.max_distance, (1000 / current_scan_mode.us_per_sample),
    //    m_angle_compensate_multiple, flip_x_axis_);
    return true;
}

void publish_scan(const double scan_time, ResponseNodeArray nodes, size_t node_count) {
    double msg_angle_min;
    double msg_angle_max;
    const bool reversed = (angle_max > angle_min);
    if (reversed) {
        /* NOTE(allenh1): the other case seems impossible? */
        msg_angle_min = M_PI - angle_max;
        msg_angle_max = M_PI - angle_min;
    } else {
        msg_angle_min = M_PI - angle_min;
        msg_angle_max = M_PI - angle_max;
    }
    const double angle_increment = (msg_angle_max - msg_angle_min) / (double)(node_count - 1);

    const double time_increment = scan_time / (double)(node_count - 1);
    const double range_min = min_distance;
    const double range_max = max_distance;

    static float ranges[MAX_SCAN_COUNT];
    static float intensities[MAX_SCAN_COUNT];

    const bool reverse_data = (!inverted_ && reversed) || (inverted_ && !reversed);
    const size_t scan_midpoint = node_count / 2;
    for (size_t i = 0; i < node_count; ++i) {
        const float read_value = (float) nodes[i].dist_mm_q2 / 4.0f / 1000;
        size_t apply_index = i;
        if (reverse_data) {
            apply_index = node_count - 1 - i;
        }
        if (flip_x_axis_) {
            if (apply_index >= scan_midpoint) {
                apply_index = apply_index - scan_midpoint;
            } else {
                apply_index = apply_index + scan_midpoint;
            }
        }
        if (read_value == 0.0) {
            // scan_msg.ranges[apply_index] = std::numeric_limits<float>::infinity();
            ranges[apply_index] = std::numeric_limits<float>::infinity();
        } else {
            // scan_msg.ranges[apply_index] = read_value;
            ranges[apply_index] = read_value;
        }
        // scan_msg.intensities[apply_index] = (float) (nodes[i].quality >> 2);
        intensities[apply_index] = (float) (nodes[i].quality >> 2);
    }

    // printf("%d %f %f %f\n", scan_count, scan_time, range_min, range_max);
    // m_publisher->publish(scan_msg);
    ufr_put(g_pub, "%f %f %f %f %f %f %f", msg_angle_min, msg_angle_max, angle_increment, time_increment, scan_time, range_min, range_max);
    ufr_put_af32(g_pub, ranges, node_count);
    ufr_put_af32(g_pub, intensities, node_count);
    ufr_send(g_pub);
}

int publish_loop() {
    u_result op_result;
    auto nodes = std::make_unique<rplidar_response_measurement_node_hq_t[]>(MAX_SCAN_COUNT);

    size_t scan_count_inout = MAX_SCAN_COUNT;
    const auto inicio = std::chrono::high_resolution_clock::now();
    op_result = m_drv->grabScanDataHq(nodes.get(), scan_count_inout);
    const auto fim = std::chrono::high_resolution_clock::now();
    const double scan_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(fim - inicio).count();
    const size_t count = scan_count_inout;

    if (op_result != RESULT_OK) {
        return -1;
    }
    op_result = m_drv->ascendScanData(nodes.get(), count);
    angle_min = deg_2_rad(0.0f);
    angle_max = deg_2_rad(359.0f);
    if (op_result == RESULT_OK) {
        if (angle_compensate_) {
            const int angle_compensate_nodes_count = 360 * m_angle_compensate_multiple;
            auto angle_compensate_nodes = std::make_unique<rplidar_response_measurement_node_hq_t[]>(
            angle_compensate_nodes_count);
            memset(
                angle_compensate_nodes.get(), 0,
                angle_compensate_nodes_count * sizeof(rplidar_response_measurement_node_hq_t));
            size_t i = 0, j = 0;
            for (; i < count; i++) {
                if (nodes[i].dist_mm_q2 != 0) {
                    int angle_compensate_offset = 0;
                    float angle = getAngle(nodes[i]);
                    int angle_value = (int)(angle * m_angle_compensate_multiple);
                    if ((angle_value - angle_compensate_offset) < 0) {angle_compensate_offset = angle_value;}
                    for (j = 0; j < m_angle_compensate_multiple; j++) {
                        angle_compensate_nodes[angle_value - angle_compensate_offset + j] = nodes[i];
                    }
                }
            }
            publish_scan(scan_duration, std::move(angle_compensate_nodes), angle_compensate_nodes_count);

        } else {
            size_t start_node = 0, end_node = 0;
            int i = 0;
            // find the first valid node and last valid node
            while (nodes[i++].dist_mm_q2 == 0) {}
            start_node = i - 1;
            i = count - 1;
            while (nodes[i--].dist_mm_q2 == 0) {}
            end_node = i + 1;

            angle_min = deg_2_rad(getAngle(nodes[start_node]));
            angle_max = deg_2_rad(getAngle(nodes[end_node]));
            auto valid = std::make_unique<rplidar_response_measurement_node_hq_t[]>(
            end_node - start_node + 1);
            for (size_t x = start_node, y = 0; x < end_node; ++x, ++y) {
                valid[y] = nodes[x];
            }
            publish_scan(scan_duration, std::move(valid), end_node - start_node + 1);
        }

    } else if (op_result == RESULT_OPERATION_FAIL) {
        // All the data is invalid, just publish them
        angle_min = deg_2_rad(0.0f);
        angle_max = deg_2_rad(359.0f);
        publish_scan(scan_duration, std::move(nodes), count);
    }

    // Success
    return 0;
}

// ============================================================================
//  Main
// ============================================================================

int main(int argc, char** argv) {
    /* parameters */
    // std::string tcp_ip_;
    // int tcp_port_ = 5000;
    const char* frame_id = "laser_frame";
    const char* topic_name = "/scan";
    const int serial_baudrate = 115200;
    const char* serial_port = (argc >= 2) ? argv[1] : "/dev/ttyUSB0";
    printf("Iniciando Rplidar on %s with baudrate %d\n", serial_port, serial_baudrate);

    g_pub = ufr_publisher_env("UFR_SCAN");
    ufr_exit_if_error(g_pub);

    /* initialize SDK */
    m_drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
    if (nullptr == m_drv) {
        printf("Failed to construct driver\n");
        return -1;
    }

    // make connection...
    if (IS_FAIL(m_drv->connect(serial_port, (_u32)serial_baudrate))) {
        printf("Error, cannot bind to the specified serial port '%s'.\n", serial_port);
        rplidar_free();
        return -1;
    }

    // get rplidar device info
    if (!getRPLIDARDeviceInfo()) {
        /* don't continue */
        printf("Error, cannot get infor from lidar '%s'.\n", serial_port);
        rplidar_free();
        return -1;
    }

    // Start motor
    start_motor();
    if (!set_scan_mode()) {
        // error 
        printf("Error, on set the scan mode\n");
        m_drv->stop();
        m_drv->stopMotor();
        RPlidarDriver::DisposeDriver(m_drv);
        exit(1);
    }

    // Main loop
    printf("Loop principal\n");
    while( ufr_loop_ok() ) {
        const int res = publish_loop();
        if ( res != 0 ) {
            printf("Error Timeout\n");
            break;
        }
    }

    // End
    printf("Desligando\n");
    stop_motor();
    rplidar_free();
    ufr_close(g_pub);
    return 0;
}