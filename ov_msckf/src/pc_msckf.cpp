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


#include "pc_msckf.h"

using namespace ov_msckf;

PCMSCKF::PCMSCKF() {
	params = parse_json(); /* json file will go here*/
	sys = new VioManager(params);
	time_buffer = 100;
	struct StateEstimate {
	    std::vector<float> pos;
	    std::vector<float> vel;
	    std::vector<float> q;
	    std::vector<float> cov_upper;
	    Eigen::Matrix<double, 6, 6> cov_full;
	};
}

PCMSCKF::StateEstimate PCMSCKF::get_state(const float t) {
	PCMSCKF::StateEstimate est;

	State* s = sys->get_state();
	Propagator* p = sys->get_propagator();

	Eigen::Matrix<double, 13, 1> state_plus;
	p->fast_state_propagate(s, t, state_plus);

	est.pos.push_back(state_plus(0));
	est.pos.push_back(state_plus(1));
	est.pos.push_back(state_plus(2));

	est.vel.push_back(state_plus(3));
	est.vel.push_back(state_plus(4));
	est.vel.push_back(state_plus(5));

	est.q.push_back(state_plus(6));
	est.q.push_back(state_plus(7));
	est.q.push_back(state_plus(8));
	est.q.push_back(state_plus(9));

	std::vector<Type*> statevars;
    statevars.push_back(s->_imu->pose()->p());
    statevars.push_back(s->_imu->pose()->q());

	est.cov_full = StateHelper::get_marginal_covariance(s,statevars);
	// only save upper diagonal
	for (int i = 0; i < 4; i+=3) {
	    est.cov_upper.push_back(est.cov_full(i, i));
	    est.cov_upper.push_back(est.cov_full(i, 1 + i));
	    est.cov_upper.push_back(est.cov_full(i, 2 + i));
	    est.cov_upper.push_back(est.cov_full(1 + i, 1 + i));
	    est.cov_upper.push_back(est.cov_full(1 + i, 2 + i));
	    est.cov_upper.push_back(est.cov_full(2 + i, 2 + i));
	}

	return est;
}

void PCMSCKF::predict_imu(const std::vector<float>& gyro, const std::vector<float>& acc, const float t) {
	Eigen::Vector3d wm, am;
	wm << gyro[0], gyro[1], gyro[2];
	am << acc[0], acc[1], acc[2];

	sys->feed_measurement_imu(t, wm, am);
}

void PCMSCKF::update_monocular(const cv::Mat& img0, const float t) {
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

void PCMSCKF::update_stereo(const cv::Mat& img0, const cv::Mat& img1, const float t0, const float t1) {
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

void PCMSCKF::end_sim() {
	delete sys;
}
