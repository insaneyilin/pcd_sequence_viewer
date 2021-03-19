/*
 * Copyright 2020, no one
 */
#include <string>
#include <vector>
#include <memory>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

class PcdSequenceViewer {
 public:
  explicit PcdSequenceViewer(const std::string &dir);
  PcdSequenceViewer() = delete;
  ~PcdSequenceViewer() = default;

  void Run();

 private:
  void KeyboardEventCallback(
      const pcl::visualization::KeyboardEvent &event, void *data);

 private:
  int num_files_ = 0;
  int cur_file_idx_ = 0;
  std::vector<std::string> filepaths_;
  bool exit_flag_ = false;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ = nullptr;
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ = nullptr;
};
