#ifndef IMAGE_PROCESSOR_NODELET_H
#define IMAGE_PROCESSOR_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <gtsam_vio/image_processor.h>

namespace vi_slam {
class ImageProcessorNodelet : public nodelet::Nodelet {
public:
  ImageProcessorNodelet() { return; }
  ~ImageProcessorNodelet() { return; }

private:
  virtual void onInit();
  ImageProcessorPtr img_processor_ptr;
};
} 

#endif

