#ifndef object_detection_H_
#define object_detection_H_

#include <iostream>

#include <math.h> 

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/passthrough.h>

using namespace std;

typedef pcl::PointXYZRGB PointType;

pcl::PointCloud<PointType>::Ptr target0_ (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr target1_ (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr best_target_reg_0_ (new pcl::PointCloud<PointType> ());

namespace point_cloud {  
  class ObjectDetection {

    public:
    ObjectDetection(const ros::NodeHandle nh);
      virtual ~ObjectDetection();
      void configure();

    private:
      ros::NodeHandle nodeHandle;
      // TOPIC INPUT     
      ros::Subscriber points2_sub_;
      ros::Subscriber world_coord_sub_;
      // TOPIC OUTPUT
      ros::Publisher  points2_pub_;
      ros::Publisher  target_pose_pub_;
      ros::Publisher  target_pose_w_pub_;
      // LAUNCH PARAMETERES
      bool   use_only_first_target_;

      bool   debug_height_;
      bool   debug_max_min_height_;
      bool   debug_best_target_;
      bool   debug_set_inclination_;
      bool   debug_registration_;
   
      bool   filter_range_;
      int    min_scene_point_size_;
      float  min_range_ , max_range_;
      float  layer_height_;
      float  incr_layer_height_;   
      float  reg_sc_th_; 
      float  max_correspnd_dist_;
      float  threshold_score_; 
      float  minimum_score_dif_; 
      std::string frame_id_;
      tf::TransformListener tf_listener_;
      // PARAMETERES
      bool   initialized_;
      bool   b_world_pub_;
      bool   ready2_publish_;   
      bool   orientation_req_PC_;
      int    it_bef_height_filter_;     
      std::string robot_frame_id_;
      tf::StampedTransform robot2camera_;
      bool robot2camera_init_;
      geometry_msgs::Pose sensor2_world_;
      Eigen::Affine3f     rot_matrix_; 
      Eigen::Affine3f     anti_rot_matrix_;      
      Eigen::Matrix4f     initial_guess_;
     
      void inputWorldCoordClb(geometry_msgs::Pose input_world_coord);

      void inputCloudClb (const sensor_msgs::PointCloud2::ConstPtr& in_cloud);

      void targetOrientationEvaluation(pcl::PointCloud<PointType>::ConstPtr height_filt_scene,
                                       Eigen::Vector4f sc_centroid, pcl::PointCloud<PointType>::Ptr& best_target_reg,
                                       float & best_score, Eigen::Affine3f & output_trans);
   
      void getMinAndMaxPoints(pcl::PointCloud<PointType>::ConstPtr scene_filtered,
                              float & sc_min_height_point, float & sc_max_height_point);

      void heightFilter(pcl::PointCloud<PointType>::ConstPtr input_cloud, 
                        pcl::PointCloud<PointType>::Ptr output_cloud,
                        float min_height_point, float max_height_point);

      bool getRobot2Camera(const std::string& camera_frame_id);

      void registrationPC(pcl::PointCloud<PointType>::ConstPtr input_target, Eigen::Matrix4f i_guess,
                          pcl::PointCloud<PointType>::ConstPtr scene,
                          float minimum_score_dif, float max_correspondence_dist,
                          pcl::PointCloud<PointType>::Ptr& reg_result, float & reg_score, Eigen::Affine3f & output_trans);

      void publishData(pcl::PointCloud<PointType>::ConstPtr reg_result, 
                       const sensor_msgs::PointCloud2::ConstPtr& in_cloud);

      void publishPose(const tf::Transform robot_to_target);
      tf::Transform matrix4fToTf(const Eigen::Matrix4f& in);
  };
}
#endif /* local_location_H_ */