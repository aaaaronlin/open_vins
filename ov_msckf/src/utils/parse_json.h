//
// Created by aaron on 8/1/20.
//

#ifndef OPEN_VINS_PARSE_JSON_H
#define OPEN_VINS_PARSE_JSON_H

#include <utils/json.hpp>
#include <core/VioManagerOptions.h>

namespace ov_msckf {
VioManagerOptions parse_json() {

  // json setup
  using json = nlohmann::json;
  json j;
  std::ifstream ifs("/home/aaron/Documents/OpenVins/settings.json");
  if (ifs.is_open()) {
    ifs >> j;
    ifs.close();
  }

  VioManagerOptions params;

  // Recording
  params.record_timing_information = j.value("do_time", false);
  params.record_timing_filepath = j.value(
      "path_time", "/home/aaron/Desktop/openvins_data/traj_timing.txt");

  // filter parameters
  params.state_options.max_clone_size = j["FilterParameters"].value(
      "max_clones", 11);
  params.state_options.max_slam_features = j["FilterParameters"].value(
      "max_slam", 50);
  params.state_options.max_slam_in_update = j["FilterParameters"].value(
      "max_slam_in_update", 25);
  params.state_options.max_msckf_in_update = j["FilterParameters"].value(
      "max_msckf_in_update", 999);
  params.state_options.feat_rep_msckf = LandmarkRepresentation::from_string(
      "GLOBAL_3D");
  params.state_options.feat_rep_slam = LandmarkRepresentation::from_string(
      "GLOBAL_3D");

  params.init_imu_thresh = j["FilterParameters"].value("init_imu_thresh", 0.12);
  params.init_window_time = j["FilterParameters"].value("up_msckf_sigma_pix",
                                                        0.5);

  // tracker/extractor
  params.use_klt = true;
  params.fast_threshold = j["TrackerParameters"].value("fast_threshold", 400);
  params.grid_x = j["TrackerParameters"].value("grid_x", 4);
  params.grid_y = j["TrackerParameters"].value("grid_y", 3);
  params.min_px_dist = j["TrackerParameters"].value("min_px_dist", 10);
  params.num_pts = j["TrackerParameters"].value("num_pts", 400);

  // ZUPT
  params.try_zupt = false;
  params.zupt_options.chi2_multipler = 1;
  params.zupt_max_velocity = 0.25;
  params.zupt_noise_multiplier = 10;

  // IMU Parameters
  params.msckf_options.sigma_pix = j["SensorParameters"].value(
      "up_msckf_sigma_pix", 1);
  params.msckf_options.chi2_multipler = j["SensorParameters"].value(
      "up_msckf_chi2_multipler", 1);
  // noise density
  params.imu_noises.sigma_w = j["SensorParameters"].value(
      "gyroscope_noise_density", 0.007955);
  params.imu_noises.sigma_a = j["SensorParameters"].value(
      "accelerometer_noise_density", 0.004334);
  // random walk
  params.imu_noises.sigma_wb = j["SensorParameters"].value(
      "gyroscope_random_walk", 0.0);
  params.imu_noises.sigma_ab = j["SensorParameters"].value(
      "accelerometer_random_walk", 0.0);

  // Camera Parameters
  params.state_options.num_cameras = j["CameraParameters"].value("num_cameras",
                                                                 1);
  for (int i = 0; i < params.state_options.num_cameras; i++) {
    bool is_fisheye = j["CameraParameters"].value(
        "cam" + std::to_string(i) + "_is_fisheye", false);
    std::vector<int> matrix_wh = j["CameraParameters"]["cam" + std::to_string(i)
        + "_wh"].get<std::vector<int>>();
    std::pair<int, int> wh(matrix_wh.at(0), matrix_wh.at(1));
    // intrinsics
    Eigen::Matrix<double, 8, 1> cam_calib;
    std::vector<double> matrix_k = j["CameraParameters"]["cam"
        + std::to_string(i) + "_k"].get<std::vector<double>>();
    std::vector<double> matrix_d = j["CameraParameters"]["cam"
        + std::to_string(i) + "_d"].get<std::vector<double>>();
    cam_calib << matrix_k.at(0), matrix_k.at(1), matrix_k.at(2), matrix_k.at(3), matrix_d
        .at(0), matrix_d.at(1), matrix_d.at(2), matrix_d.at(3);
    // transform
    Eigen::Matrix4d T_CtoI;
    std::vector<double> matrix_TCtoI = j["CameraParameters"]["T_C"
        + std::to_string(i) + "toI"].get<std::vector<double>>();
    T_CtoI << matrix_TCtoI.at(0), matrix_TCtoI.at(1), matrix_TCtoI.at(2), matrix_TCtoI
        .at(3), matrix_TCtoI.at(4), matrix_TCtoI.at(5), matrix_TCtoI.at(6), matrix_TCtoI
        .at(7), matrix_TCtoI.at(8), matrix_TCtoI.at(9), matrix_TCtoI.at(10), matrix_TCtoI
        .at(11), matrix_TCtoI.at(12), matrix_TCtoI.at(13), matrix_TCtoI.at(14), matrix_TCtoI
        .at(15);
    std::vector<double> camera_rotation;
    Eigen::Matrix<double, 4, 1> cam_q = rot_2_quat(T_CtoI.block(0, 0, 3, 3));
    camera_rotation.push_back(cam_q[0]);
    camera_rotation.push_back(cam_q[1]);
    camera_rotation.push_back(cam_q[2]);
    camera_rotation.push_back(cam_q[3]);
    Eigen::Matrix<double, 7, 1> cam_eigen;
    cam_eigen.block(0, 0, 4, 1) = rot_2_quat(
        T_CtoI.block(0, 0, 3, 3).transpose());
    cam_eigen.block(4, 0, 3, 1) = -T_CtoI.block(0, 0, 3, 3).transpose()
        * T_CtoI.block(0, 3, 3, 1);

    params.camera_fisheye.insert( { i, is_fisheye });
    params.camera_intrinsics.insert( { i, cam_calib });
    params.camera_extrinsics.insert( { i, cam_eigen });
    params.camera_wh.insert( { i, wh });
    params.camera_rotation.insert( { i, camera_rotation });

  }

  return params;

}
}
#endif //OPEN_VINS_PARSE_JSON_H
