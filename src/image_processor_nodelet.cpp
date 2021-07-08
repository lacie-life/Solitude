#include <vi_slam_ros/image_processor_nodelet.h>

namespace vi_slam {
void ImageProcessorNodelet::onInit() {
  img_processor_ptr.reset(new ImageProcessor(getPrivateNodeHandle()));
  if (!img_processor_ptr->initialize()) {
    ROS_ERROR("Cannot initialize Image Processor...");
    return;
  }
  return;
}

PLUGINLIB_EXPORT_CLASS(gtsam_vio::ImageProcessorNodelet, nodelet::Nodelet);

} // end namespace gtsam_vio

