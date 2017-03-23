#include <object_detection_nodelet.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(point_cloud, object_detection_nodelet, point_cloud::ObjectDetectionNodelet, nodelet::Nodelet);

namespace point_cloud {

  void ObjectDetectionNodelet::onInit() {
    NODELET_INFO("[point_cloud] Initializing object detection");

    ros::NodeHandle nh = getMTPrivateNodeHandle();
    sj = new point_cloud::ObjectDetection(nh);
  }

}
