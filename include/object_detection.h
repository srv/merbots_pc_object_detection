#ifndef object_detection_H_
#define object_detection_H_

#include <iostream>

#include <math.h> 

#include <tf/transform_broadcaster.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/passthrough.h>

using namespace std;

typedef pcl::PointXYZRGB PointType;

pcl::PointCloud<PointType>::Ptr target0_ (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr target1_ (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr target2_ (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr target3_ (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr previous_target (new pcl::PointCloud<PointType> ());

// TODO Modify by local PC
pcl::PointCloud<PointType>::Ptr original_scene (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr reg_result (new pcl::PointCloud<PointType> ()); 

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
      // LAUNCH PARAMETERES
      bool   debug_detected_object;
      bool   debug_height;
      bool   debug_max_min_height;
      bool   debug_best_target;
      bool   debug_set_inclination;
      bool   user_def_plane;
      bool   filter_range_;
      int    min_scene_point_size;
      float    rot_x, rot_y, rot_z;
      float min_range_ , max_range_;
      float  layer_height;
      float  incr_layer_height;   
      float  reg_sc_th; 
      float  height_reg_sc_th;
      float  threshold_score_; 
      float  minimum_score_dif_; 
      // PARAMETERES
      bool   initialized;
      bool   b_world_pub;
      bool   ready2_publish;   
      bool   orientation_req_PC;
      int    it_bef_height_filter;     
      geometry_msgs::Pose world_tf;
      Eigen::Affine3f rot_matrix; 
      Eigen::Affine3f anti_rot_matrix;      
      Eigen::Matrix4f   initial_guess;
      Eigen::Vector4f   m_centroid; 
      Eigen::Vector4f   tar1_centroid; 
      Eigen::Vector4f   tar2_centroid; 
      Eigen::Vector4f   tar3_centroid; 
    
      void inputWorldCoordClb(geometry_msgs::Pose input_world_coord);

      void inputCloudClb (const sensor_msgs::PointCloud2::ConstPtr& in_cloud);

      void targetOrientationEvaluation(pcl::PointCloud<PointType>::ConstPtr height_filt_scene,
                                       Eigen::Vector4f sc_centroid, pcl::PointCloud<PointType>::Ptr& best_target_reg,
                                       float & best_score);
   
      void getMinAndMaxPoints(pcl::PointCloud<PointType>::ConstPtr scene_filtered,
                              float & sc_min_height_point, float & sc_max_height_point);

      void heightFilter(pcl::PointCloud<PointType>::ConstPtr input_cloud, 
                        pcl::PointCloud<PointType>::Ptr output_cloud,
                        float min_height_point, float max_height_point);

      void registrationPC(pcl::PointCloud<PointType>::ConstPtr points_with_normals_src,
                          pcl::PointCloud<PointType>::ConstPtr scene,
                          float minimum_score_dif, float max_correspondence_dist,
                          pcl::PointCloud<PointType>::Ptr& reg_result, float & reg_score );

      void detectedTargetPositionAdapt(pcl::PointCloud<PointType>::ConstPtr reg_result, 
                           pcl::PointCloud<PointType>::ConstPtr scene_filtered,
                           const sensor_msgs::PointCloud2::ConstPtr& in_cloud,
                           Eigen::Vector3f & target_pos, Eigen::Quaternionf & target_quat);
  };
}

#endif /* local_location_H_ */