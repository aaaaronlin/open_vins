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

#ifndef OV_MSCKF_PC_MSCKF_H
#define OV_MSCKF_PC_MSCKF_H


#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "utils/parse_json.h"
#include "state/State.h"

namespace ov_msckf {

	class PCMSCKF {

	public:
		struct StateEstimate {
		    std::vector<float> pos;
		    std::vector<float> vel;
		    std::vector<float> q;
		    std::vector<float> cov_upper;
		    Eigen::Matrix<double, 6, 6> cov_full;
		};
		StateEstimate get_state(const float t);

		PCMSCKF();
		/* move state forward with imu measurement */
		void predict_imu(const std::vector<float>& gyro, const std::vector<float>& acc, const float t);

		/* move state forward with one camera measurement */
		void update_monocular(const cv::Mat& img0, const float t);

		/* move state forward with two camera measurements */
		void update_stereo(const cv::Mat& img0, const cv::Mat& img1, const float t0, const float t1);

		void end_sim();

		VioManagerOptions get_params() {
			return params;
		}

	protected:
		VioManager* sys;
		VioManagerOptions params;

		double time_buffer;
		cv::Mat img0_buffer, img1_buffer;

	};
}

#endif
