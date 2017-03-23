#include "object_detection.h"

int main(int argc, char** argv) {

  ros::init (argc, argv, "object_detection");
  ros::NodeHandle nh("~");

  point_cloud::ObjectDetection sj(nh);
  //sj.configure();
  ros::spin();
  return 0;
}
