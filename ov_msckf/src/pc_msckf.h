/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "utils/parse_json.h"

using namespace ov_msckf;

VioManager* sys;

double time_buffer = -1;
cv::Mat img0_buffer, img1_buffer;

void predict_imu(const std::vector<float>& gyro, const std::vector<float>& acc, const float t) {
    Eigen::Vector3d wm, am;
    wm << gyro[0], gyro[1], gyro[2];
    am << acc[0], acc[1], acc[2];

    sys->feed_measurement_imu(t, wm, am);
}

void update_monocular(const cv::Mat& img0, const float t) {
    // fill buffer
    if(img0_buffer.rows == 0) {
        time_buffer = t;
        img0_buffer = img0.clone();
        return;
    }

    sys->feed_measurement_monocular(time_buffer, img0_buffer, 0);

    time_buffer = t;
    img0_buffer = img0.clone();
}

void update_stereo(const cv::Mat& img0, const cv::Mat& img1, const float t0, const float t1) {
    // fill buffer
    if(img0_buffer.rows == 0 || img1_buffer.rows == 0) {
        time_buffer = t0;
        img0_buffer = img0.clone();
        time_buffer = t1;
        img1_buffer = img1.clone();
        return;
    }

    sys->feed_measurement_stereo(time_buffer, img0_buffer, img1_buffer, 0, 1);

    time_buffer = t0;
    img0_buffer = img0.clone();
    time_buffer = t1;
    img1_buffer = img1.clone();
}
