#include "object_detection.h"

namespace point_cloud{

  ObjectDetection::ObjectDetection(const ros::NodeHandle nh):
      nodeHandle(nh)
  {
    configure();
  }

  ObjectDetection::~ObjectDetection(){
  }

  void ObjectDetection::configure(){
    //** PARAMETERS **
    //TARGET INPUT
    float rot_x, rot_y, rot_z;

    std::string target0_filename;
    std::string target1_filename;

    nodeHandle.param<std::string>("frame_id", frame_id_, "camera");
    nodeHandle.param<std::string>("target_0", target0_filename, "vertices14_0.pcd");
    nodeHandle.param<std::string>("target_1", target1_filename, "vertices14_90.pcd");
    nodeHandle.param<std::string>("target_name", target_name_, "caduf");
    nodeHandle.param("use_only_first_target", use_only_first_target_, false);


    // DEBUG
    nodeHandle.param("debug_height", debug_height_, false);
    nodeHandle.param("debug_set_inclination", debug_set_inclination_, false);
    nodeHandle.param("debug_max_min_height", debug_max_min_height_, false);
    nodeHandle.param("debug_best_target", debug_best_target_, false);
    nodeHandle.param("debug_registration", debug_registration_, false);
    // COMPENSATE SENSOR INCLINATION   
    nodeHandle.param("user_def_plane_rot_x", rot_x, 0.0f);
    nodeHandle.param("user_def_plane_rot_y",rot_y, 0.0f);
    nodeHandle.param("user_def_plane_rot_z", rot_z, 0.0f);
    // FILTER RANGE FOR OUTLIERS
    nodeHandle.param("filter_range", filter_range_, false);
    nodeHandle.param("min_range", min_range_, 0.2f);
    nodeHandle.param("max_range", max_range_, 10.0f);
    // LAYER PARAMETERS
    nodeHandle.param("layer_height", layer_height_, 0.05f);
    nodeHandle.param("incr_layer_height", incr_layer_height_, 0.01f);
    nodeHandle.param("min_scene_point_size", min_scene_point_size_, 1000);
    // REGISTRATION PARAMETERS
    nodeHandle.param("reg_sc_th", reg_sc_th_, 0.000009f);
    nodeHandle.param("max_correspnd_dist", max_correspnd_dist_, 0.0000075f);
    nodeHandle.param("threshold_score_", threshold_score_, 0.00005f);
    nodeHandle.param("minimum_score_dif_", minimum_score_dif_, 0.00001f);
    // DYNAMIC PARAMETERS
    it_bef_height_filter_ = 0;
    initialized_ = false;
    ready2_publish_ = false;
    b_world_pub_ = false;

    //Initialize, if desired by user, rotation matrix for previous scene rotation.
   
    if (rot_x != 0 || rot_y != 0 || rot_z != 0){
      orientation_req_PC_ = true;
      float rad_rot_x = rot_x * M_PI / 180;
      float rad_rot_y = rot_y * M_PI / 180;
      float rad_rot_z = rot_z * M_PI / 180;

      rot_matrix_ = Eigen::Affine3f::Identity();
      rot_matrix_.rotate (Eigen::AngleAxisf (-rad_rot_x, Eigen::Vector3f::UnitX()));
      rot_matrix_.rotate (Eigen::AngleAxisf (-rad_rot_y, Eigen::Vector3f::UnitY()));
      rot_matrix_.rotate (Eigen::AngleAxisf (-rad_rot_z, Eigen::Vector3f::UnitZ()));

      anti_rot_matrix_ = Eigen::Affine3f::Identity();
      anti_rot_matrix_.rotate (Eigen::AngleAxisf (rad_rot_x, Eigen::Vector3f::UnitX()));
      anti_rot_matrix_.rotate (Eigen::AngleAxisf (rad_rot_y, Eigen::Vector3f::UnitY()));
      anti_rot_matrix_.rotate (Eigen::AngleAxisf (rad_rot_z, Eigen::Vector3f::UnitZ()));
    }
    else (orientation_req_PC_ = false);
   
    // Load targets
    if (pcl::io::loadPCDFile (target0_filename, *best_target_reg_0_) < 0){
      std::cout << "Error loading target-0 cloud." << std::endl;
      return;
    }

    if (pcl::io::loadPCDFile (target1_filename, *target1_) < 0){
      std::cout << "Error loading target-1 cloud." << std::endl;
      return;
    }

    // Input
    world_coord_sub_ = nodeHandle.subscribe("/world_coord", 1, &ObjectDetection::inputWorldCoordClb, this);
    points2_sub_ = nodeHandle.subscribe("/input_cloud", 1, &ObjectDetection::inputCloudClb, this);

    // Output
    target_pose_pub_= nodeHandle.advertise<geometry_msgs::Pose> ("object_pose_world", 1);
    points2_pub_ = nodeHandle.advertise<sensor_msgs::PointCloud2>("out", 1);
  }

  void ObjectDetection::heightFilter(pcl::PointCloud<PointType>::ConstPtr input_cloud,
                                     pcl::PointCloud<PointType>::Ptr output_cloud,
                                     float min_height_point, float max_height_point){

    pcl::PassThrough<PointType> pass;
    pass.setInputCloud (input_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (min_height_point, max_height_point);
    pass.filter (*output_cloud);
  }

  void ObjectDetection::registrationPC(pcl::PointCloud<PointType>::ConstPtr input_target, Eigen::Matrix4f i_guess,
                                       pcl::PointCloud<PointType>::ConstPtr scene, float minimum_score_dif,
                                       float max_correspondence_dist, pcl::PointCloud<PointType>::Ptr& reg_result,
                                       float & reg_score, Eigen::Affine3f & output_trans){

    pcl::PointCloud<PointType>::Ptr reg_target_0 (new pcl::PointCloud<PointType> ());

    pcl::transformPointCloud (*input_target, *reg_target_0, i_guess);

    if (debug_registration_){
      pcl::visualization::PointCloudColorHandlerCustom<PointType> blue_color(reg_target_0, 0, 0, 255);
      pcl::visualization::PointCloudColorHandlerCustom<PointType> red_color(input_target, 255, 0, 0);
      pcl::visualization::PointCloudColorHandlerCustom<PointType> green_color(best_target_reg_0_, 0, 255, 0);
      pcl::visualization::PointCloudColorHandlerCustom<PointType> ambar_color(scene, 0, 255, 255);
      pcl::visualization::PointCloudColorHandlerCustom<PointType> _color(scene, 255, 255, 255);

      pcl::visualization::PCLVisualizer viewer5 ("DEBUG Registration");
      viewer5.addCoordinateSystem (1.0);
      viewer5.addPointCloud<PointType> (best_target_reg_0_, green_color, "Target best1 ");
      viewer5.addPointCloud<PointType> (input_target, red_color, "Scene filtered");
      viewer5.addPointCloud<PointType> (reg_target_0, blue_color, "Target best2 ");
      viewer5.addPointCloud<PointType> (scene, ambar_color, "Target best3 ");
      while(!viewer5.wasStopped()) viewer5.spinOnce (1);
    }

    reg_score = 1.0;

    pcl::IterativeClosestPointNonLinear<PointType, PointType> reg;
    reg.setTransformationEpsilon (1e-6); //1e-6
    reg.setMaxCorrespondenceDistance (max_correspondence_dist);  //5.5 good initial 0.1
    reg.setInputSource (reg_target_0);
    reg.setInputTarget (scene);
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    pcl::PointCloud<PointType>::Ptr var_point_cloud (new pcl::PointCloud<PointType> ());
    pcl::copyPointCloud(*reg_target_0, *var_point_cloud);
    reg.setMaximumIterations (2);
    reg_result = var_point_cloud ;
    float score;
    float prev_score = 0;

    for (int i = 0; i < 15; ++i) {
      var_point_cloud = reg_result;
      reg.setInputSource (var_point_cloud);
      reg.align (*reg_result);
      //accumulate transformation between each Iteration
      Ti = reg.getFinalTransformation () * Ti;

      //if the difference between this transformation and the previous one
      //is smaller than the threshold, refine the process by reducing
      //the maximal correspondence distance
      /* if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
        reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);*/

      prev = reg.getLastIncrementalTransformation ();
      score = reg.getFitnessScore();

      //std::cout << "ITERATION=" << i << "; SCORE=" << score << "; Incremental score:" << prev_score - score << "\n" << std::endl;
      if (sqrt ((prev_score - score)*(prev_score - score)) < minimum_score_dif) {
        Eigen::Matrix4f output (reg.getFinalTransformation());
        reg_score = score;
        output_trans.matrix() = Ti * i_guess ;

        if (debug_registration_){
          pcl::PointCloud<PointType>::Ptr reg_target_1 (new pcl::PointCloud<PointType> ());
          pcl::PointCloud<PointType>::Ptr reg_target_2 (new pcl::PointCloud<PointType> ());
          Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
          std::cout << "SCORE" << score << "\n" << std::endl;
          transform_3.matrix() =  Ti;
          pcl::visualization::PointCloudColorHandlerCustom<PointType> blue_color(reg_target_0, 0, 0, 255);
          pcl::visualization::PointCloudColorHandlerCustom<PointType> red_color(input_target, 255, 0, 0);
          pcl::visualization::PointCloudColorHandlerCustom<PointType> green_color(best_target_reg_0_, 0, 255, 0);
          pcl::visualization::PointCloudColorHandlerCustom<PointType> ambar_color(scene, 0, 255, 255);
          pcl::visualization::PointCloudColorHandlerCustom<PointType> _color(scene, 255, 255, 255);
          pcl::transformPointCloud (*reg_target_0, *reg_target_1, transform_3);
          pcl::transformPointCloud (*best_target_reg_0_, *reg_target_2, output_trans);
          pcl::visualization::PCLVisualizer viewer6 ("DEBUG END Registration");
          viewer6.addCoordinateSystem (1.0);
          viewer6.addPointCloud<PointType> (best_target_reg_0_, green_color, "Target best1 ");
          viewer6.addPointCloud<PointType> (reg_result, red_color, "Scene filtered");
          viewer6.addPointCloud<PointType> (reg_target_1, blue_color, "Target best2 ");
          viewer6.addPointCloud<PointType> (scene, ambar_color, "Target best3 ");
          viewer6.addPointCloud<PointType> (reg_target_2, _color, "Target best4 ");
           while(!viewer6.wasStopped()) viewer6.spinOnce (1);
       }
        return;
      }
      prev_score = score;
    }
    reg_score = score;
  }

  void ObjectDetection::getMinAndMaxPoints(pcl::PointCloud<PointType>::ConstPtr input_cloud,
                                           float & sc_min_height_point, float & sc_max_height_point){

    PointType min_values_PC, max_values_PC;
    pcl::getMinMax3D(*input_cloud, min_values_PC, max_values_PC);
    sc_max_height_point = min_values_PC.z;
    sc_min_height_point = max_values_PC.z;

    if (debug_max_min_height_){
      pcl::PointCloud<PointType>::Ptr min_max_points_cloud (new pcl::PointCloud<PointType> ());

      PointType min;
      min.x = min_values_PC.x;
      min.y = min_values_PC.y;
      min.z = min_values_PC.z;
      min_max_points_cloud->push_back(min);

      PointType max;
      max.x = max_values_PC.x;
      max.y = max_values_PC.y;
      max.z = max_values_PC.z;
      min_max_points_cloud->push_back(max);

      std::cout << "Min_height_point_main =  " << sc_min_height_point << "\n" <<std:: endl;
      std::cout << "Max_height_point_main =  " << sc_max_height_point << "\n" <<std:: endl;

      pcl::visualization::PCLVisualizer viewer2 ("MAX MIN PLANE HEIGHT");
      viewer2.addCoordinateSystem (1.0);
      pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(input_cloud, 255, 0, 0);
      viewer2.addPointCloud<PointType> (input_cloud, single_color, "Planes");
      pcl::visualization::PointCloudColorHandlerCustom<PointType> green(min_max_points_cloud, 0, 255, 0);
      viewer2.addPointCloud<PointType> (min_max_points_cloud, green, "point");

      pcl::ModelCoefficients coeffs;
      coeffs.values.push_back (0);
      coeffs.values.push_back (0);
      coeffs.values.push_back (1);
      coeffs.values.push_back (-sc_max_height_point);
      viewer2.addPlane (coeffs, "plane");

      pcl::ModelCoefficients coeffs_2;
      coeffs_2.values.push_back (0);
      coeffs_2.values.push_back (0);
      coeffs_2.values.push_back (1);
      coeffs_2.values.push_back (-(sc_min_height_point));
      viewer2.addPlane (coeffs_2, "plane_2");

      while(!viewer2.wasStopped()) viewer2.spinOnce (1);
    }
  }

  // INITIALIZATION:: Find the best target orientation
  void ObjectDetection::targetOrientationEvaluation(pcl::PointCloud<PointType>::ConstPtr height_filt_scene,
                                                    Eigen::Vector4f s_centroid, pcl::PointCloud<PointType>::Ptr& best_target_reg,
                                                    float & best_score, Eigen::Affine3f & output_target_trans){

    float score[2];
    pcl::PointCloud<PointType>::Ptr reg_result0 (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr reg_result1 (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr target_aligned (new pcl::PointCloud<PointType> ());
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    Eigen::Affine3f output_trans = Eigen::Affine3f::Identity();
    
    transform_2.translation() << s_centroid[0], s_centroid[1], s_centroid[2] ;
    Eigen::Matrix4f i_guess = transform_2.matrix();

    // TARGET #1
    registrationPC(best_target_reg_0_, i_guess, height_filt_scene, max_correspnd_dist_, 5.5 , reg_result0, score[0], output_trans);
    pcl::copyPointCloud(*reg_result0, *best_target_reg);
    output_target_trans = output_trans;
    best_score = score[0];
    // TARGET #2
    if (!use_only_first_target_){
      registrationPC(target1_, i_guess, height_filt_scene, max_correspnd_dist_, 5.5 , reg_result1, score[1], output_trans);
      if (score[1] < best_score){
        pcl::copyPointCloud(*reg_result1, *best_target_reg);
        pcl::copyPointCloud(*target1_, *best_target_reg_0_);
        best_score = score[1];
        output_target_trans = output_trans;
      }
    }
 
    if (debug_best_target_){
      std::cout << "SCORE 0 Degrees   =  " << score[0] << "--"<< "\n" <<std:: endl;
      std::cout << "SCORE 90 Degrees  =  " << score[1] << "--"<<"\n" <<std:: endl;
      std::cout << "AND THE BEST IS   =  " << best_score << "\n" << std:: endl;

      pcl::visualization::PointCloudColorHandlerCustom<PointType> red_color(height_filt_scene, 255, 0, 0);
      pcl::visualization::PointCloudColorHandlerCustom<PointType> green_color(best_target_reg, 0, 255, 0);

      pcl::visualization::PCLVisualizer viewer5 ("TARGET AT 0 DEG");
      viewer5.addPointCloud<PointType> (height_filt_scene, red_color, "Scene filtered");
      viewer5.addPointCloud<PointType> (best_target_reg, green_color, "Target best ");

      pcl::visualization::PCLVisualizer viewer6 ("TARGET AT 90 DEG");
      viewer6.addPointCloud<PointType> (height_filt_scene, red_color, "Scene filtered");
      viewer6.addPointCloud<PointType> (reg_result1, green_color, "Target 0 ");

      pcl::visualization::PCLVisualizer viewer9 ("BEST TARGET");
      viewer9.addPointCloud<PointType> (height_filt_scene, red_color, "Scene filtered");
      viewer9.addPointCloud<PointType> (best_target_reg, green_color, "Target best ");

      while(!viewer5.wasStopped()) viewer5.spinOnce (1);
      while(!viewer6.wasStopped()) viewer6.spinOnce (1);
      while(!viewer9.wasStopped()) viewer9.spinOnce (1);
    }

  }

  void ObjectDetection::publishPose(const std_msgs::Header header,
                                    const tf::Transform cam_to_target) {
    // Publish geometry message from world frame id
    if (target_pose_pub_.getNumSubscribers() > 0) {
      try {
        ros::Time now = ros::Time::now();
        tf::StampedTransform world2camera;
        tf_listener_.waitForTransform("world",
                                      header.frame_id,
                                      now, ros::Duration(1.0));
        tf_listener_.lookupTransform("world",
            header.frame_id, now, world2camera);

        // Compose the message
        static tf::TransformBroadcaster world_target;
        geometry_msgs::Pose pose_msg;
        tf::Transform world2target = world2camera * cam_to_target;

        // Fix orientation
        tf::Matrix3x3 rot = world2target.getBasis();
        double r, p, y;
        rot.getRPY(r, p, y);
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, y);
        world2target.setRotation(q);

        world_target.sendTransform(tf::StampedTransform(world2target, header.stamp, "world", target_name_));

        pose_msg.position.x = world2target.getOrigin().x();
        pose_msg.position.y = world2target.getOrigin().y();
        pose_msg.position.z = world2target.getOrigin().z();
        pose_msg.orientation.x = world2target.getRotation().x();
        pose_msg.orientation.y = world2target.getRotation().y();
        pose_msg.orientation.z = world2target.getRotation().z();
        pose_msg.orientation.w = world2target.getRotation().w();


        target_pose_pub_.publish(pose_msg);
      } catch (tf::TransformException ex) {
        ROS_WARN_STREAM("Cannot find the tf between " <<
          "world frame id and camera. " << ex.what());
      }
    }
  }

  tf::Transform ObjectDetection::matrix4fToTf(const Eigen::Matrix4f& in) {
    tf::Vector3 t_out;
    t_out.setValue(static_cast<double>(in(0,3)),
                   static_cast<double>(in(1,3)),
                   static_cast<double>(in(2,3)));

    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(in(0,0)), static_cast<double>(in(0,1)), static_cast<double>(in(0,2)),
                  static_cast<double>(in(1,0)), static_cast<double>(in(1,1)), static_cast<double>(in(1,2)),
                  static_cast<double>(in(2,0)), static_cast<double>(in(2,1)), static_cast<double>(in(2,2)));

    tf::Quaternion q_out;
    tf3d.getRotation(q_out);
    tf::Transform out(q_out, t_out);
    return out;
  }

  void ObjectDetection::publishData(pcl::PointCloud<PointType>::ConstPtr object_detected, 
                                                    const sensor_msgs::PointCloud2::ConstPtr& in_cloud){

    sensor_msgs::PointCloud2 out_cloud;
    toROSMsg(*object_detected, out_cloud);
    out_cloud.header = in_cloud->header;
    points2_pub_.publish(out_cloud);

    Eigen::Matrix3f initial_guess_rot;
    initial_guess_rot(0,0) = initial_guess_ (0,0);
    initial_guess_rot(0,1) = initial_guess_ (0,1);
    initial_guess_rot(0,2) = initial_guess_ (0,2);
    initial_guess_rot(1,0) = initial_guess_ (1,0);
    initial_guess_rot(1,1) = initial_guess_ (1,1);
    initial_guess_rot(1,2) = initial_guess_ (1,2);
    initial_guess_rot(2,0) = initial_guess_ (2,0);
    initial_guess_rot(2,1) = initial_guess_ (2,1);
    initial_guess_rot(2,2) = initial_guess_ (2,2);
    Eigen::Quaternionf target_quat (initial_guess_rot);

    // PUBLISH POSE TF
    tf::Quaternion q;

    q.setRPY (-22.5 * M_PI / 180,0,0);
    tf::Transform perp_rot (q);
    tf::Transform cam_rot =  matrix4fToTf(rot_matrix_.matrix());
    static tf::TransformBroadcaster br_target;
    tf::Transform tf_sensor2object;
    tf_sensor2object.setOrigin( tf::Vector3(initial_guess_(0,3), initial_guess_(1,3), initial_guess_(2,3)) );
    tf_sensor2object.setRotation( tf::Quaternion (target_quat.x(), target_quat.y(), target_quat.z(), target_quat.w()));
    tf_sensor2object = tf_sensor2object * perp_rot*cam_rot;
    //br_target.sendTransform(tf::StampedTransform(tf_sensor2object, ros::Time::now(), in_cloud->header.frame_id, "amphoraJP"));

    // PUBLSIH POSE STAMPED
    publishPose(in_cloud->header, tf_sensor2object);

  }

  void ObjectDetection::inputWorldCoordClb(geometry_msgs::Pose input_world_coord){
    b_world_pub_ = true;
    sensor2_world_ = input_world_coord;
  }

  void ObjectDetection::inputCloudClb (const sensor_msgs::PointCloud2::ConstPtr& in_cloud){
    pcl::PointCloud<PointType>::Ptr input_scene (new pcl::PointCloud<PointType> ());
    fromROSMsg(*in_cloud, *input_scene);

    if (input_scene->points.size() == 0){
      std::cout << "WARNING ObjectDetection:: No points in point cloud \n" <<std:: endl;
      return;
    }
    std::cout << " Received Input cloud with: " << input_scene->points.size()<< " points \n" <<std:: endl;
    pcl::PointCloud<PointType>::Ptr reg_result (new pcl::PointCloud<PointType> ()); 
    // **
    // TRACKING PART. If the target has been found by height evaluation just ICP is performed.
    // **If target is not found during 5 input clouds then the scene will be evaluated by height.
    //
    if (initialized_){
      float reg_initial_score;

      Eigen::Affine3f output_trans = Eigen::Affine3f::Identity();
      registrationPC(best_target_reg_0_, initial_guess_, input_scene, reg_sc_th_, 5.5, reg_result, reg_initial_score, output_trans);
      std::cout << "SCORE:" << reg_initial_score << "\n "<<  std::endl;
      if (reg_initial_score < threshold_score_){
        Eigen::Matrix4f output_trans_4f;
        output_trans_4f = output_trans.matrix();
        initial_guess_ =  output_trans_4f;
        //std::cout << "MATHCES  FOUND, Score = " << reg_initial_score << "\n" <<std:: endl;      

        // TOPIC PUB
        // Just Publish filtered cloud after a good registered PC
        //
        if (ready2_publish_){
          publishData(reg_result, in_cloud);   
        }
        ready2_publish_ = true;
        return;
      }

      else{
        it_bef_height_filter_ = it_bef_height_filter_ + 1;

        if (it_bef_height_filter_ <5) return;
      }
    }

    // FUNCTION TO TRANSFORM THE SCENE FLOOR PERP. TO THE Z SENSOR AXIS
    if (orientation_req_PC_){

      pcl::PointCloud<PointType>::Ptr original_scene (new pcl::PointCloud<PointType> ());
      if (debug_set_inclination_) (pcl::copyPointCloud(*input_scene, *original_scene));

      pcl::transformPointCloud (*input_scene, *input_scene, rot_matrix_);
      Eigen::Matrix4f rotation_matrix4;
      rotation_matrix4 = rot_matrix_.matrix();
      initial_guess_ = initial_guess_* rot_matrix_.matrix() ;

      if (debug_set_inclination_){
        pcl::visualization::PCLVisualizer viewer16 ("Debug set inclination");
        viewer16.addCoordinateSystem (1.0);
        pcl::visualization::PointCloudColorHandlerCustom<PointType> green_color(input_scene, 0, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<PointType> red_color(original_scene, 255, 0, 0);
        
        viewer16.addPointCloud<PointType> (input_scene, green_color, "input_scene1");
        viewer16.addPointCloud<PointType> (original_scene, red_color, "original_scene1");
        while(!viewer16.wasStopped()) viewer16.spinOnce (1);
      }
    }

    // HEIGHT FILTER TO BE APPLIED TO REJECT HEIGHT OUTLIERS FROM SENSOR
    if (filter_range_) (heightFilter(input_scene, input_scene, min_range_, max_range_));

    // **
    // DETECTION PART: Registration the target & scene at different heights of the scene
    // **
    ready2_publish_ = false;
    float sc_min_height_point, sc_max_height_point;
    getMinAndMaxPoints(input_scene, sc_min_height_point, sc_max_height_point);

    float top_height = sc_max_height_point;
    float bottom_height = sc_max_height_point + layer_height_;

    float min_loop_score = 1;

    while (bottom_height < sc_min_height_point){

      pcl::PointCloud<PointType>::Ptr height_filt_scene (new pcl::PointCloud<PointType> ());
      heightFilter(input_scene, height_filt_scene, top_height, bottom_height);

      if ((height_filt_scene->points.size()) > min_scene_point_size_){

        Eigen::Vector4f s_centroid;
        // Move target to align the centroid
        pcl::compute3DCentroid(*height_filt_scene, s_centroid);
        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        transform_2.translation() << s_centroid[0], s_centroid[1], s_centroid[2];

        Eigen::Affine3f output_trans = Eigen::Affine3f::Identity();
        float reg_score;
        if (!initialized_) targetOrientationEvaluation(height_filt_scene, s_centroid, reg_result, reg_score, output_trans);

        else {
          Eigen::Matrix4f i_guess = transform_2.matrix();
          registrationPC(best_target_reg_0_, i_guess, height_filt_scene, max_correspnd_dist_, 5.5 , reg_result, reg_score, output_trans);
        }
        // Correspondence evaluation between Scene & target

        if (debug_height_){
          std::cout << "SCORE = " << reg_score << " at height bottom_height" << bottom_height << " - " << top_height <<std:: endl;
          pcl::visualization::PCLVisualizer viewer12 ("Debug HEIGHT");

          pcl::visualization::PointCloudColorHandlerCustom<PointType> blue_color(input_scene, 0, 0, 255);
          pcl::visualization::PointCloudColorHandlerCustom<PointType> green_color(height_filt_scene, 0, 255, 0);
          pcl::visualization::PointCloudColorHandlerCustom<PointType> red_color(reg_result, 255, 0, 0);
          viewer12.addCoordinateSystem (1.0);

          viewer12.addPointCloud<PointType> (input_scene, blue_color, "input_scene1");
          viewer12.addPointCloud<PointType> (height_filt_scene, green_color, "height_filt_scene1");
          viewer12.addPointCloud<PointType> (reg_result, red_color, "reg_result1");
          while(!viewer12.wasStopped()) viewer12.spinOnce (1);
        }

        if (reg_score < min_loop_score)(min_loop_score = reg_score);

        if (reg_score < threshold_score_){
          std::cout << "MATCH FOUND, score= " << reg_score << "" <<std:: endl;
          initialized_ = true;
          Eigen::Vector3f target_pos;
          Eigen::Quaternionf target_quat;

          Eigen::Matrix4f output_trans_4f;
          output_trans_4f = output_trans.matrix();

          if (orientation_req_PC_){                       
            Eigen::Matrix4f anti_rot_matrix_4f;
            anti_rot_matrix_4f = anti_rot_matrix_.matrix();            
            initial_guess_ = anti_rot_matrix_4f * output_trans_4f;
          }

          else initial_guess_ = output_trans_4f;

          it_bef_height_filter_ = 0;
          return;
        }
      }

      top_height = top_height + incr_layer_height_;
      bottom_height = bottom_height + incr_layer_height_;
    }

    if (!initialized_) (std::cout << "Initial target not found...  \n"  <<std:: endl);
    std::cout << "Minimum Loop score = " << min_loop_score << "" <<std:: endl;
    return;
  }
}
