//
// Created by aaron on 8/1/20.
//

#ifndef OPEN_VINS_PARSE_JSON_H
#define OPEN_VINS_PARSE_JSON_H

#include <json.hpp>
#include <core/VioManagerOptions.h>

namespace ov_msckf {
    VioManagerOptions parse_json() {

        VioManagerOptions params;

        // Recording
        params.record_timing_information = false;
        params.record_timing_filepath = "/home/aaron/Desktop/openvins_data/traj_timing.txt";

        // filter parameters
        params.state_options.max_clones = 11;
        params.state_options.max_slam = 50;
        params.state_options.max_slam_in_update = 25;
        params.state_options.max_msckf_in_update = 999;
        params.state_options.feat_rep_msckf = "GLOBAL_3D";
        params.state_options.feat_rep_slam = "GLOBAL_3D";
        // Global gravity
        std::vector<double> gravity = {0.0, 0.0, 9.81};
        params.gravity << gravity.at(0),gravity.at(1),gravity.at(2);

        // tracker/extractor
        params.use_klt = true;
        params.fast_threshold = 15;
        params.grid_x = 4;
        params.grid_y = 3;
        params.min_px_dst = 10;
        params.num_pts = 400;

        // IMU Parameters
        params.init_imu_thresh = 0.12;
        params.init_window_time = 0.5;

        params.msckf_options.sigma_pix = 1;
        params.msckf_options.chi2_multiplier = 1;
        // noise density
        params.imu_noises.sigma_w = 0.007855;
        params.imu_noises.sigma_a = 0.004334;
        // random walk
        params.imu_noises.sigma_wb = 0.0;
        params.imu_noises.sigma_ab = 0.0;

        // freq
        params.sim_freq_cam = 30.0;
        params.sim_freq_imu = 100.0;

        // Camera Parameters
        // TODO add multiple cameras
        bool is_fisheye = false;
        std::vector<int> matrix_wd = {640,480};
        std::pair<int,int> wh(matrix_wh.at(0),matrix_wh.at(1));
        // intrinsics
        Eigen::Matrix<double,8,1> cam_calib;
        std::vector<double> matrix_k = {320.0,320.0,320.0,240.0};
        std::vector<double> matrix_d = {0.0,0.0,0.0,0.0};
        cam_calib << matrix_k.at(0),matrix_k.at(1),matrix_k.at(2),matrix_k.at(3),matrix_d.at(0),matrix_d.at(1),matrix_d.at(2),matrix_d.at(3);
        // transform
        Eigen::Matrix4d T_CtoI;
        std::vector<double> matrix_TCtoI = {-1.0,0.0,0.0,0.0,
                                            0.0,0.0,-1.0,0.0,
                                            0.0,-1.0,0.0,0.5,
                                            0.0,0.0,0.0,1.0};
        Eigen::Matrix<double,7,1> cam_eigen;
        cam_eigen.block(0,0,4,1) = rot_2_quat(T_CtoI.block(0,0,3,3).transpose());
        cam_eigen.block(4,0,3,1) = -T_CtoI.block(0,0,3,3).transpose()*T_CtoI.block(0,3,3,1);

        params.camera_fisheye.insert({0, is_fisheye});
        params.camera_intrinsics.insert({0, cam_calib});
        params.camera_extrinsics.insert({0, cam_eigen});
        params.camera_wh.insert({0, wh});

        return params;

    }
}
#endif //OPEN_VINS_PARSE_JSON_H
