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

#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "alignment/AlignTrajectory.h"
#include "alignment/AlignUtils.h"
#include "utils/Loader.h"
#include "utils/Math.h"
#include "utils/Colors.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

// Will plot the xy 3d position of the pose trajectories
void plot_xy_positions(const std::string &name, const std::string &color, const std::vector<Eigen::Matrix<double,7,1>> &poses) {

    // Paramters for our line
    std::map<std::string, std::string> params;
    params.insert({"label", name});
    params.insert({"linestyle", "-"});
    params.insert({"color", color});

    // Create vectors of our x and y axis
    std::vector<double> x, y;
    for(size_t i=0; i<poses.size(); i++) {
        x.push_back(poses.at(i)(0));
        y.push_back(poses.at(i)(1));
    }

    // Finally plot
    matplotlibcpp::plot(x, y, params);

}

// Will plot the 3d position of the pose trajectories
void plot_position(const std::string &name, const std::string &color, const std::vector<double> &times, const std::vector<Eigen::Matrix<double,7,1>> &poses, const int &axis) {

    // Paramters for our line
    std::map<std::string, std::string> params;
    params.insert({"label", name});
    params.insert({"linestyle", "-"});
    params.insert({"color", color});

    // Create vectors of our x and y axis
    std::vector<double> time, pos_axis;
    for(size_t i=0; i<poses.size(); i++) {
        time.push_back(times.at(i));
        pos_axis.push_back(poses.at(i)(axis));
    }

    // Finally plot
    matplotlibcpp::plot(time, pos_axis, params);

}

// Will plot the 3d vel of the pose trajectories
void plot_vel(const std::string &name, const std::string &color, const std::vector<double> &times, const std::vector<Eigen::Matrix<double,3,1>> &vel, const int &axis) {

    // Paramters for our line
    std::map<std::string, std::string> params;
    params.insert({"label", name});
    params.insert({"linestyle", "-"});
    params.insert({"color", color});

    // Create vectors of our x and y axis
    std::vector<double> time, vel_axis;
    for(size_t i=0; i<vel.size(); i++) {
        time.push_back(times.at(i));
        vel_axis.push_back(vel.at(i)(axis));
    }

    // Finally plot
    matplotlibcpp::plot(time, vel_axis, params);

}

// Will plot the 3d position of the pose trajectories
void plot_euler(const std::string &name, const std::string &color, const std::vector<double> &times, const std::vector<Eigen::Matrix<double,3,1>> &angles, const int &axis) {

    // Paramters for our line
    std::map<std::string, std::string> params;
    params.insert({"label", name});
    params.insert({"linestyle", "-"});
    params.insert({"color", color});

    // Create vectors of our x and y axis
    std::vector<double> time, pos_axis;
    for(size_t i=0; i<angles.size(); i++) {
        time.push_back(times.at(i));
        pos_axis.push_back(angles.at(i)(axis));
    }

    // Finally plot
    matplotlibcpp::plot(time, pos_axis, params);

}

void plot_sigma_bounds(const std::string &name, const std::string &color, const std::vector<double> &times, const std::vector<Eigen::Matrix<double,7,1>> &poses, const std::vector<Eigen::Matrix3d> &cov_pos, const int &axis) {

  // Paramters for our line
  std::map<std::string, std::string> params;
  params.insert({"linestyle", "-"});
  params.insert({"color", color});

  std::vector<double> time, pos, sigma, upper, lower;
  for (size_t i=0; i<poses.size(); i++) {
    time.push_back(times.at(i));
    pos.push_back(poses.at(i)(axis));
    sigma.push_back(sqrt(abs(cov_pos.at(i)(axis, axis))));

    upper.push_back(pos.at(i) + sigma.at(i));
    lower.push_back(pos.at(i) - sigma.at(i));
  }

  matplotlibcpp::plot(time, upper, params);
  matplotlibcpp::plot(time, lower, params);

  params.insert({"label", "1 std bound"});
  matplotlibcpp::fill_between(time, lower, upper, params);


}


void plot_sigma_bounds_euler(const std::string &name, const std::string &color, const std::vector<double> &times, const std::vector<Eigen::Matrix<double,3,1>> &angles, const std::vector<Eigen::Matrix3d> &cov_pos, const int &axis) {

    // Paramters for our line
    std::map<std::string, std::string> params;
    params.insert({"linestyle", "-"});
    params.insert({"color", color});

    std::vector<double> time, angle, sigma, upper, lower;
    for (size_t i=0; i<angles.size(); i++) {
        time.push_back(times.at(i));
        angle.push_back(angles.at(i)(axis));
        sigma.push_back(sqrt(abs(cov_pos.at(i)(axis, axis))));

        upper.push_back(angle.at(i) + sigma.at(i)*180/M_PI);
        lower.push_back(angle.at(i) - sigma.at(i)*180/M_PI);
    }

    matplotlibcpp::plot(time, upper, params);
    matplotlibcpp::plot(time, lower, params);

    params.insert({"label", "1 std bound"});
    matplotlibcpp::fill_between(time, lower, upper, params);


}
#endif

int main(int argc, char **argv) {

  // Ensure we have a path
  if (argc < 3) {
    printf(
    RED "ERROR: Please specify a align mode and trajectory file\n" RESET);
    printf(
        RED "ERROR: ./plot_trajectories <align_mode> <file_gt.txt> <file_est1.txt> ...  <file_est9.txt>\n" RESET);
    printf(
        RED "ERROR: rosrun ov_eval plot_trajectories <align_mode> <file_gt.txt> <file_est1.txt> ...  <file_est9.txt>\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Read in all our trajectories from file
  std::vector<std::string> names;
  std::vector<std::vector<double>> times;
  std::vector<std::vector<Eigen::Matrix<double, 7, 1>>> poses;
  std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> vel;
  std::vector<std::vector<Eigen::Matrix3d>> cov_ori, cov_pos;

  for (int i = 2; i < argc; i++) {

    // Read in trajectory data
    std::vector<double> times_temp;
    std::vector<Eigen::Matrix<double, 7, 1>> poses_temp;
    std::vector<Eigen::Matrix<double, 3, 1>> vel_temp;
    std::vector<Eigen::Matrix3d> cov_ori_temp, cov_pos_temp;
    ov_eval::Loader::load_data(argv[i], times_temp, poses_temp, vel_temp, cov_ori_temp,
                               cov_pos_temp);

    // Align all the non-groundtruth trajectories to the base one
    if (i > 2) {

      // Intersect timestamps
      std::vector<double> gt_times_temp(times.at(0));
      std::vector<Eigen::Matrix<double, 7, 1>> gt_poses_temp(poses.at(0));
      ov_eval::AlignUtils::perform_association(0, 0.02, times_temp,
                                               gt_times_temp, poses_temp,
                                               gt_poses_temp);

      // Return failure if we didn't have any common timestamps
      if (poses_temp.size() < 2) {
        printf(
            RED "[TRAJ]: unable to get enough common timestamps between trajectories.\n" RESET);
        printf(
            RED "[TRAJ]: does the estimated trajectory publish the rosbag timestamps??\n" RESET);
        std::exit(EXIT_FAILURE);
      }

      // Perform alignment of the trajectories
      Eigen::Matrix3d R_ESTtoGT;
      Eigen::Vector3d t_ESTinGT;
      double s_ESTtoGT;
      ov_eval::AlignTrajectory::align_trajectory(poses_temp, gt_poses_temp,
                                                 R_ESTtoGT, t_ESTinGT,
                                                 s_ESTtoGT, argv[1]);

      // Debug print to the user
      Eigen::Vector4d q_ESTtoGT = ov_eval::Math::rot_2_quat(R_ESTtoGT);
      printf(
          "[TRAJ]: q_ESTtoGT = %.3f, %.3f, %.3f, %.3f | p_ESTinGT = %.3f, %.3f, %.3f | s = %.2f\n",
          q_ESTtoGT(0), q_ESTtoGT(1), q_ESTtoGT(2), q_ESTtoGT(3), t_ESTinGT(0),
          t_ESTinGT(1), t_ESTinGT(2), s_ESTtoGT);

      // Finally lets calculate the aligned trajectories
      std::vector<Eigen::Matrix<double, 7, 1>> est_poses_aignedtoGT;
      for (size_t j = 0; j < gt_times_temp.size(); j++) {
        Eigen::Matrix<double, 7, 1> pose_ESTinGT;
        pose_ESTinGT.block(0, 0, 3, 1) = s_ESTtoGT * R_ESTtoGT
            * poses_temp.at(j).block(0, 0, 3, 1) + t_ESTinGT;
        pose_ESTinGT.block(3, 0, 4, 1) = ov_eval::Math::quat_multiply(
            poses_temp.at(j).block(3, 0, 4, 1), ov_eval::Math::Inv(q_ESTtoGT));
        est_poses_aignedtoGT.push_back(pose_ESTinGT);
      }

      // Overwrite our poses with the corrected ones
      poses_temp = est_poses_aignedtoGT;

    }

    // Debug print the length stats
    boost::filesystem::path path(argv[i]);
    std::string name = path.stem().string();
    double length = ov_eval::Loader::get_total_length(poses_temp);
    printf("[COMP]: %d poses in %s => length of %.2f meters\n",
           (int) times_temp.size(), name.c_str(), length);

    // Save this to our arrays
    names.push_back(name);
    times.push_back(times_temp);
    poses.push_back(poses_temp);
    vel.push_back(vel_temp);

    cov_ori.push_back(cov_ori_temp);
    cov_pos.push_back(cov_pos_temp);

  }

#ifdef HAVE_PYTHONLIBS

    // Colors that we are plotting
    std::vector<std::string> colors = {"black","blue","red","green","cyan","magenta"};
    //assert(algo_rpe.size() <= colors.size()*linestyle.size());

    // Zero our time arrays
    double starttime = (times.at(0).empty())? 0 : times.at(0).at(0);
    double endtime = (times.at(0).empty())? 0 : times.at(0).at(times.at(0).size()-1);
    for(size_t i=0; i<times.size(); i++) {
        for(size_t j=0; j<times.at(i).size(); j++) {
            times.at(i).at(j) -= starttime;
        }
    }

    // Plot this figure
    matplotlibcpp::figure_size(1000, 1200);

    // Plot the position trajectories
    std::vector<std::string> axis = {"x", "y", "z"};

    for (int j=0; j<3; j++) {
        matplotlibcpp::subplot(3, 1, j+1);
        for (size_t i = 0; i < times.size(); i++) {
            plot_position(names.at(i), colors.at(i), times.at(i), poses.at(i), j);
        }
        plot_sigma_bounds(names.at(0), colors.at(4), times.at(0), poses.at(0), cov_pos.at(0), j);

        matplotlibcpp::xlabel("timestamp (sec)");
        matplotlibcpp::ylabel(axis.at(j) + "-axis (m)");
        matplotlibcpp::xlim(0.0, endtime - starttime);
        matplotlibcpp::legend();
    }

    // Plot this figure
    matplotlibcpp::figure_size(1000, 1200);

    for (int j=0; j<3; j++) {
        matplotlibcpp::subplot(3, 1, j+1);
        for (size_t i = 0; i < times.size(); i++) {
            plot_vel(names.at(i), colors.at(i), times.at(i), vel.at(i), j);
        }

        matplotlibcpp::xlabel("timestamp (sec)");
        matplotlibcpp::ylabel("vel_" + axis.at(j) + "-axis (m)");
        matplotlibcpp::xlim(0.0, endtime - starttime);
        matplotlibcpp::legend();

    }

    // Convert Quat to  Euler Angles
    std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> angles;
    // go through all traj
    for (size_t i = 0; i < times.size(); i++) {
        std::vector<Eigen::Matrix<double, 3, 1>> angle;
        // go through one dataset
        for (size_t t = 0; t < poses.at(i).size(); t++) {
            Eigen::Quaterniond q(poses.at(i).at(t)(6), poses.at(i).at(t)(3), poses.at(i).at(t)(4), poses.at(i).at(t)(5));
            //Eigen::Vector3d e = q.toRotationMatrix().eulerAngles(2, 1, 0); // y-p-r
            double roll = atan2(2*(q.w()*q.x() + q.y()*q.z()), 1 - 2*(pow(q.x(), 2) + pow(q.y(), 2)));
            double pitch = asin(2*(q.w()*q.y() - q.z()*q.x()));
            double yaw = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(pow(q.y(), 2) + pow(q.z(), 2)));
            Eigen::Vector3d e_xyz(roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI); // reorganize by rot_x, rot_y, rot_z to follow OpenVins covariance notation
            angle.push_back(e_xyz);
        }
        angles.push_back(angle);
    }

    matplotlibcpp::figure_size(1000, 1200);
    std::vector<std::string> angle_name = {"roll (x)", "pitch (y)", "yaw (z)"};

    // Plot the euler angles
    for (int j=0; j<3; j++) {
        matplotlibcpp::subplot(3, 1, j+1);
        for (size_t i = 0; i < times.size(); i++) {
            plot_euler(names.at(i), colors.at(i), times.at(i), angles.at(i), j);
        }
        plot_sigma_bounds_euler(names.at(0), colors.at(4), times.at(0), angles.at(0), cov_ori.at(0), j);

        matplotlibcpp::xlabel("timestamp (sec)");
        matplotlibcpp::ylabel(angle_name.at(j) + " (deg)");
        matplotlibcpp::xlim(0.0, endtime - starttime);
        matplotlibcpp::legend();
    }


    matplotlibcpp::show(true);
#endif

  // Done!
  return EXIT_SUCCESS;

}

