#ifndef IO__GALAXY__GALAXY_HPP
#define IO__GALAXY__GALAXY_HPP

#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>

#include "../camera.hpp"
#include "include/GxIAPI.h"

namespace io {

class Galaxy : public CameraBase {
public:
  Galaxy(double exposure_ms, double gain, const std::string &vid_pid);
  ~Galaxy() override;

  void read(cv::Mat &img,
            std::chrono::steady_clock::time_point &timestamp) override;

private:
  void initializeLibrary();
  void openDevice(const std::string &vid_pid);
  void configureCamera(double exposure_ms, double gain);
  void startAcquisition();
  void stopAcquisition();
  void closeDevice();
  bool convertFrameToMat(PGX_FRAME_BUFFER frame_buffer, cv::Mat &img);

  GX_DEV_HANDLE device_handle_;
  bool is_open_;
  bool is_streaming_;
};

} // namespace io

#endif // IO__GALAXY__GALAXY_HPP