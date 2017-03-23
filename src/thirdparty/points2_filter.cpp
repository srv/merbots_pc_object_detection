#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

class Points2Fiter {
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber points2_sub_;
  ros::Subscriber altitude_sub_;
  ros::Publisher  points2_pub_;

  // Filters
  bool filter_nan_;
  bool filter_range_;
  bool filter_outliers_;
  bool filter_size_;
  bool filter_altitude_;
  bool filter_voxelgrid_;

  // Operational parameters
  double min_range_;
  double max_range_;
  int min_size_;
  double voxel_size_;
  double min_altitude_;
  double max_altitude_;

  // Other
  std::string node_name_;
  double altitude_;
  double altitude_stamp_;

 public:
  Points2Fiter() : nh_private_("~") {
    node_name_ = ros::this_node::getName();

    // Params
    nh_private_.param("filter_nan",       filter_nan_,        false);
    nh_private_.param("filter_range",     filter_range_,      false);
    nh_private_.param("filter_outliers",  filter_outliers_,   false);
    nh_private_.param("filter_size",      filter_size_,       false);
    nh_private_.param("filter_altitude",  filter_altitude_,   false);
    nh_private_.param("filter_voxelgrid", filter_voxelgrid_,  false);

    nh_private_.param("min_range",        min_range_,         0.5);
    nh_private_.param("max_range",        max_range_,         5.0);
    nh_private_.param("min_size",         min_size_,          200);
    nh_private_.param("voxel_size",       voxel_size_,        0.025);

    nh_private_.param("min_altitude",     min_altitude_,      0.5);
    nh_private_.param("max_altitude",     max_altitude_,      6.0);

    // Input
    points2_sub_ = nh_.subscribe("/in", 1, &Points2Fiter::Points2Cb, this);
    if (filter_altitude_) {
      altitude_sub_ = nh_.subscribe("/altitude", 1,
        &Points2Fiter::Points2Cb, this);
      altitude_stamp_ = -1.0;
    }

    // Output
    points2_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("out", 1);
  }

  void AltitudeCb(const sensor_msgs::RangeConstPtr& altitude) {
    altitude_ = altitude->range;
    altitude_stamp_ = altitude->header.stamp.toSec();
  }

  void Points2Cb(const sensor_msgs::PointCloud2::ConstPtr& in_cloud) {
    // Only process if subscribers in the output cloud
    if (points2_pub_.getNumSubscribers() == 0)
      return;

    // Copy
    PointCloud::Ptr cloud(new PointCloud);
    fromROSMsg(*in_cloud, *cloud);

    if (cloud->points.size() == 0)
      return;

    // Altitude filtering
    if (filter_altitude_) {
      if (altitude_stamp_ < 0.0) {
        ROS_WARN_STREAM("[" << node_name_ << "]: You specify an altitude " <<
          " filtering but I'm not receiving altitude measures...");
        return;
      } else {
        const double max_time_diff = 2.0;
        double time_diff =
          fabs(altitude_stamp_ - in_cloud->header.stamp.toSec());
        if ( time_diff > max_time_diff ) {
          ROS_WARN_STREAM("[" << node_name_ << "]: The time difference " <<
            " between altitude measures and pointclouds is " <<
            time_diff << " (maximum: " << max_time_diff << ")");
          return;
        } else {
          if (altitude_ > max_altitude_ || altitude_ < min_altitude_) {
            return;
          }
        }
      }
    }

    // NAN filtering
    if (filter_nan_) {
      std::vector<int> indicies;
      pcl::removeNaNFromPointCloud(*cloud, *cloud, indicies);
    }

    if (cloud->points.size() == 0)
      return;

    // Range filtering
    if (filter_range_) {
      pcl::PassThrough<Point> pass;
      pass.setFilterFieldName("z");
      pass.setFilterLimits(min_range_, max_range_);
      pass.setInputCloud(cloud);
      pass.filter(*cloud);
    }

    if (cloud->points.size() == 0)
      return;

    // Statistical outlier removal
    if (filter_outliers_) {
      pcl::RadiusOutlierRemoval<Point> outrem;
      outrem.setInputCloud(cloud);
      outrem.setRadiusSearch(0.2);
      outrem.setMinNeighborsInRadius(5);
      outrem.filter(*cloud);
    }

    if (cloud->points.size() == 0)
      return;

    // Filter size
    if (filter_size_) {
      if (static_cast<int>(cloud->points.size()) < min_size_) {
        ROS_DEBUG_STREAM("[" << node_name_ << "]: The cloud didn't pass " <<
          " the min size filter. Cloud size: " <<
          cloud->points.size() << ", min_size: " << min_size_ << ".");
        return;
      }
    }

    if (cloud->points.size() == 0)
      return;

    // Voxel grid
    if (filter_voxelgrid_) {
      pcl::ApproximateVoxelGrid<Point> grid;
      grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
      grid.setDownsampleAllData(true);
      grid.setInputCloud(cloud);
      grid.filter(*cloud);
    }

    if (cloud->points.size() == 0)
      return;

    // Publish filtered cloud
    sensor_msgs::PointCloud2 out_cloud;
    toROSMsg(*cloud, out_cloud);
    out_cloud.header = in_cloud->header;
    points2_pub_.publish(out_cloud);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "points2_filter");
  Points2Fiter node;
  ros::spin();
  return 0;
}

