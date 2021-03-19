/*
 * Copyright 2020, no one
 */
#include "pcd_sequence_viewer.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <string.h>
#include <limits.h>
#include <stddef.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <algorithm>
#include <thread>
#include <chrono>

#include <pcl/io/pcd_io.h>

static int GetPCDFilePathsInDir(const std::string &dir,
    std::vector<std::string> *filepaths) {
  DIR *dp = nullptr;
  struct dirent *dirp = nullptr;
  if ((dp = opendir(dir.c_str())) == nullptr) {
    std::cerr << "GetPCDFilePathsInDir error: " << errno
        << " dir: " << dir << std::endl;
    return errno;
  }

  filepaths->clear();
  while ((dirp = readdir(dp)) != nullptr) {
    if (std::string(dirp->d_name).find(".pcd") != std::string::npos) {
      filepaths->push_back(dir + "/" + std::string(dirp->d_name));
    }
  }
  closedir(dp);
  std::sort(filepaths->begin(), filepaths->end());
  return 0;
}

PcdSequenceViewer::PcdSequenceViewer(const std::string &dir) {
  cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  viewer_.reset(new pcl::visualization::PCLVisualizer("PCD Viewer"));

  GetPCDFilePathsInDir(dir, &filepaths_);
  num_files_ = static_cast<int>(filepaths_.size());
  cur_file_idx_ = 0;
}

void PcdSequenceViewer::Run() {
  if (num_files_ <= 0) {
    std::cerr << "no pcd files.\n";
    return;
  }

  viewer_->addPointCloud<pcl::PointXYZ>(cloud_, "frame cloud");
  viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "frame cloud");
  viewer_->registerKeyboardCallback(&PcdSequenceViewer::KeyboardEventCallback,
      *this, nullptr);
  viewer_->setBackgroundColor(0, 0, 0);
  viewer_->addCoordinateSystem(1.0);
  viewer_->initCameraParameters();

  while (!viewer_->wasStopped() && !exit_flag_) {
    viewer_->spinOnce();
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return;
}

void PcdSequenceViewer::KeyboardEventCallback(
    const pcl::visualization::KeyboardEvent &event, void *data) {
  std::string pressed_key = event.getKeySym();
  if (num_files_ <= 0) {
    std::cout << "no pcd files.\n";
    return;
  }

  const int old_file_idx = cur_file_idx_;
  if (event.keyDown()) {
    if (pressed_key == "n") {
      ++(cur_file_idx_);
      if (cur_file_idx_ >= num_files_) {
        cur_file_idx_ = 0;
      }
    } else if (pressed_key == "p") {
      --(cur_file_idx_);
      if (cur_file_idx_ < 0) {
        cur_file_idx_ = num_files_ - 1;
      }
    } else if (pressed_key == "q") {
      exit_flag_ = true;
      return;
    }
    if (old_file_idx == cur_file_idx_) {
      return;
    }
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(
        filepaths_[cur_file_idx_], *cloud_) == -1) {
      std::cerr << "loadPCDFile failed. filepath: "
          << filepaths_[cur_file_idx_] << '\n';
      return;
    }
    std::cout << "loadPCDFile success. filepath: "
        << filepaths_[cur_file_idx_]
        << " number of points: " << cloud_->width * cloud_->height << '\n';
    viewer_->spinOnce();
    viewer_->updatePointCloud<pcl::PointXYZ>(cloud_, "frame cloud");
  }
}
