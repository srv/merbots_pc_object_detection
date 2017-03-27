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
    std::string target0_filename;
    std::string target1_filename;
    std::string target2_filename;
    std::string target3_filename;
    nodeHandle.param<std::string>("frame_id", frame_id, "camera");
    nodeHandle.param<std::string>("target_0", target0_filename, "vertices14_0.pcd");
    nodeHandle.param<std::string>("target_1", target1_filename, "vertices14_90.pcd");

    // DEBUG
    nodeHandle.param("debug_detected_object", debug_detected_object, false);
    nodeHandle.param("debug_height", debug_height, false);
    nodeHandle.param("debug_set_inclination", debug_set_inclination, false);
    nodeHandle.param("debug_max_min_height", debug_max_min_height, false);
    nodeHandle.param("debug_best_target", debug_best_target, false);
    nodeHandle.param("debug_registration", debug_registration, false);
    // COMPENSATE SENSOR INCLINATION
    nodeHandle.param("user_def_plane", user_def_plane, true);
    nodeHandle.param("user_def_plane_rot_x", rot_x, 0.0f);
    nodeHandle.param("user_def_plane_rot_y",rot_y, 0.0f);
    nodeHandle.param("user_def_plane_rot_z", rot_z, 0.0f);
    // FILTER RANGE FOR OUTLIERS
    nodeHandle.param("filter_range", filter_range_, false);
    nodeHandle.param("min_range", min_range_, 0.2f);
    nodeHandle.param("max_range", max_range_, 10.0f);
    // LAYER PARAMETERS
    nodeHandle.param("layer_height", layer_height, 0.05f);
    nodeHandle.param("incr_layer_height", incr_layer_height, 0.01f);
    nodeHandle.param("min_scene_point_size", min_scene_point_size, 1000);
    // REGISTRATION PARAMETERS
    nodeHandle.param("reg_sc_th", reg_sc_th, 0.000009f);
    nodeHandle.param("height_reg_sc_th", height_reg_sc_th, 0.0000075f);
    nodeHandle.param("threshold_score_", threshold_score_, 0.00005f);
    nodeHandle.param("minimum_score_dif_", minimum_score_dif_, 0.00001f);
    // DYNAMIC PARAMETERS
    new_initial_guess.setIdentity();
    it_bef_height_filter = 0;
    initialized = false;
    ready2_publish = false;
    b_world_pub = false;


    //Initialize, if desired by user, rotation matrix for previous scene rotation.
    if (user_def_plane == true){
      if (rot_x != 0 || rot_y != 0 || rot_z != 0){
        orientation_req_PC = true;
        float rad_rot_x = rot_x * M_PI / 180;
        float rad_rot_y = rot_y * M_PI / 180;
        float rad_rot_z = rot_z * M_PI / 180;

        rot_matrix = Eigen::Affine3f::Identity();
        rot_matrix.rotate (Eigen::AngleAxisf (-rad_rot_x, Eigen::Vector3f::UnitX()));
        rot_matrix.rotate (Eigen::AngleAxisf (-rad_rot_y, Eigen::Vector3f::UnitY()));
        rot_matrix.rotate (Eigen::AngleAxisf (-rad_rot_z, Eigen::Vector3f::UnitZ()));

        anti_rot_matrix = Eigen::Affine3f::Identity();
        anti_rot_matrix.rotate (Eigen::AngleAxisf (rad_rot_x, Eigen::Vector3f::UnitX()));
        anti_rot_matrix.rotate (Eigen::AngleAxisf (rad_rot_y, Eigen::Vector3f::UnitY()));
        anti_rot_matrix.rotate (Eigen::AngleAxisf (rad_rot_z, Eigen::Vector3f::UnitZ()));
      }
      else (orientation_req_PC = false);
    }

    // Load targets and obtain the centroid of each one
    if (pcl::io::loadPCDFile (target0_filename, *previous_target) < 0){
      std::cout << "Error loading target-0 cloud." << std::endl;
      return;
    }

    if (pcl::io::loadPCDFile (target1_filename, *target1_) < 0){
      std::cout << "Error loading target-1 cloud." << std::endl;
      return;
    }

    pcl::copyPointCloud(*previous_target, *best_target_reg_0);

    // Input
    world_coord_sub_ = nodeHandle.subscribe("/world_coord", 1, &ObjectDetection::inputWorldCoordClb, this);
    points2_sub_ = nodeHandle.subscribe("/input_cloud", 1, &ObjectDetection::inputCloudClb, this);

    // Output
    target_pose_pub_ = nodeHandle.advertise<geometry_msgs::PoseStamped> ("object_pose", 1);
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

    // i_guess(2,3) = i_guess(2,3) + 0.055;
    pcl::transformPointCloud (*input_target, *reg_target_0, i_guess);

    if (debug_registration){
      pcl::visualization::PointCloudColorHandlerCustom<PointType> blue_color(reg_target_0, 0, 0, 255);
      pcl::visualization::PointCloudColorHandlerCustom<PointType> red_color(input_target, 255, 0, 0);
      pcl::visualization::PointCloudColorHandlerCustom<PointType> green_color(best_target_reg_0, 0, 255, 0);
      pcl::visualization::PointCloudColorHandlerCustom<PointType> ambar_color(scene, 0, 255, 255);
      pcl::visualization::PointCloudColorHandlerCustom<PointType> _color(scene, 255, 255, 255);

      pcl::visualization::PCLVisualizer viewer5 ("DEBUG Registration");
      viewer5.addCoordinateSystem (1.0);
      viewer5.addPointCloud<PointType> (best_target_reg_0, green_color, "Target best1 ");
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

      std::cout << "ITERATION=" << i << "; SCORE=" << score << "; Incremental score:" << prev_score - score << "\n" << std::endl;
      if (sqrt ((prev_score - score)*(prev_score - score)) < minimum_score_dif) {
        Eigen::Matrix4f output (reg.getFinalTransformation());
        reg_score = score;

        pcl::PointCloud<PointType>::Ptr reg_target_1 (new pcl::PointCloud<PointType> ());
        pcl::PointCloud<PointType>::Ptr reg_target_2 (new pcl::PointCloud<PointType> ());
        Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();

        output_trans.matrix() = Ti * i_guess ;

        if (debug_registration){
          std::cout << "SCORE" << score << "\n" << std::endl;
          transform_3.matrix() =  Ti;
          pcl::visualization::PointCloudColorHandlerCustom<PointType> blue_color(reg_target_0, 0, 0, 255);
          pcl::visualization::PointCloudColorHandlerCustom<PointType> red_color(input_target, 255, 0, 0);
          pcl::visualization::PointCloudColorHandlerCustom<PointType> green_color(best_target_reg_0, 0, 255, 0);
          pcl::visualization::PointCloudColorHandlerCustom<PointType> ambar_color(scene, 0, 255, 255);
          pcl::visualization::PointCloudColorHandlerCustom<PointType> _color(scene, 255, 255, 255);
          pcl::transformPointCloud (*reg_target_0, *reg_target_1, transform_3);
          pcl::transformPointCloud (*best_target_reg_0, *reg_target_2, output_trans);
          pcl::visualization::PCLVisualizer viewer6 ("DEBUG END Registration");
          viewer6.addCoordinateSystem (1.0);
          viewer6.addPointCloud<PointType> (best_target_reg_0, green_color, "Target best1 ");
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

    if (debug_max_min_height){
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
    registrationPC(previous_target, i_guess, height_filt_scene, height_reg_sc_th, 5.5 , reg_result0, score[0], output_trans);
    pcl::copyPointCloud(*reg_result0, *best_target_reg);
    pcl::copyPointCloud(*previous_target, *best_target_reg_0);
    output_target_trans = output_trans;
    best_score = score[0];
    // TARGET #2
    registrationPC(target1_, i_guess, height_filt_scene, height_reg_sc_th, 5.5 , reg_result1, score[1], output_trans);
    if (score[1] < best_score){
      pcl::copyPointCloud(*reg_result1, *best_target_reg);
      pcl::copyPointCloud(*target1_, *best_target_reg_0);
      best_score = score[1];
      output_target_trans = output_trans;
    }

    if (debug_best_target){
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
  // TODO MODIFY THE NAME FROM detectedTargetPositionAdapt TO TARGET FOUND LOCATION TRANSFORMATION OR SOMTHING LIKE THIS
  void ObjectDetection::detectedTargetPositionAdapt(pcl::PointCloud<PointType>::ConstPtr object_detected,
                                        Eigen::Vector3f & target_pos, Eigen::Quaternionf & target_quat ){

    pcl::copyPointCloud(*object_detected, *previous_target);

    Eigen::Matrix3f initial_guess_rot;
    target_pos(0) = initial_guess(0,3);
    target_pos(1) = initial_guess(1,3);
    target_pos(2) = initial_guess(2,3);

    initial_guess_rot(0,0) = initial_guess (0,0);
    initial_guess_rot(0,1) = initial_guess (0,1);
    initial_guess_rot(0,2) = initial_guess (0,2);
    initial_guess_rot(1,0) = initial_guess (1,0);
    initial_guess_rot(1,1) = initial_guess (1,1);
    initial_guess_rot(1,2) = initial_guess (1,2);
    initial_guess_rot(2,0) = initial_guess (2,0);
    initial_guess_rot(2,1) = initial_guess (2,1);
    initial_guess_rot(2,2) = initial_guess (2,2);

    Eigen::Quaternionf target_quat2 (initial_guess_rot);
    target_quat = target_quat2;
  }

  void ObjectDetection::inputWorldCoordClb(geometry_msgs::Pose input_world_coord){
    b_world_pub = true;
    world_tf = input_world_coord;
  }

  void ObjectDetection::inputCloudClb (const sensor_msgs::PointCloud2::ConstPtr& in_cloud){
    pcl::PointCloud<PointType>::Ptr input_scene (new pcl::PointCloud<PointType> ());
    fromROSMsg(*in_cloud, *input_scene);

    if (input_scene->points.size() == 0){
      std::cout << "WARNING ObjectDetection:: No points in point cloud \n" <<std:: endl;
      return;
    }
    std::cout << " Received Input cloud with: " << input_scene->points.size()<< " points \n" <<std:: endl;
    // **
    // TRACKING PART. If the target has been found by height evaluation just ICP is performed.
    // **If target is not found during 5 input clouds then the scene will be evaluated by height.
    //
    if (initialized){
      float reg_initial_score;

      Eigen::Affine3f output_trans = Eigen::Affine3f::Identity();
      registrationPC(best_target_reg_0, initial_guess, input_scene, reg_sc_th, 5.5, reg_result, reg_initial_score, output_trans);
      std::cout << "SCORE:" << reg_initial_score << "\n "<<  std::endl;
      if (reg_initial_score < threshold_score_){
        Eigen::Matrix4f output_trans_4f;
        output_trans_4f = output_trans.matrix();
        initial_guess =  output_trans_4f;
        //std::cout << "MATHCES  FOUND, Score = " << reg_initial_score << "\n" <<std:: endl;

        Eigen::Vector3f target_pos;
        Eigen::Quaternionf target_quat;
        detectedTargetPositionAdapt(reg_result, target_pos, target_quat);

        // TOPIC PUB
        // Just Publish filtered cloud after a good registered PC
        //
        if (ready2_publish){
          if (b_world_pub){
            std::cout << "WORLD TRANSFORMATION WILL NOT BE APPLYED, CHECK THE CODE .- ready2_publish - \n"<< std::endl;
            //TODO apply transformation (trans/rot -- world) to pose output pub and target PC pub
            //Eigen::Vector3f trans_target2_world = world_tf.pose - target_pos;
            //Eigen::Quaternionf rot_target2_world = world_tf.orientation * target_quat
            //pcl::transformPointCloud (*reg_result, *reg_result, trans_target2_world, rot_target2_world);
            //target_pos = trans_target2_world;
            //target_quat = rot_target2_world;
          }

          sensor_msgs::PointCloud2 out_cloud;
          toROSMsg(*reg_result, out_cloud);
          out_cloud.header = in_cloud->header;
          points2_pub_.publish(out_cloud);

          // PUBLISH POSE TF
          static tf::TransformBroadcaster br;
          tf::Transform transform;
          transform.setOrigin( tf::Vector3(target_pos(0,3), target_pos(1,3), target_pos(2,3)) );
          transform.setRotation( tf::Quaternion (target_quat.x(), target_quat.y(), target_quat.z(), target_quat.w()));
          br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, "target_detected"));

          // PUBLSIH POSE STAMPED
          geometry_msgs::PoseStamped pose_pub;

          pose_pub.header = in_cloud->header;
          pose_pub.pose.position.x = target_pos.x();
          pose_pub.pose.position.y = target_pos.y();
          pose_pub.pose.position.z = target_pos.z();
          pose_pub.pose.orientation.x = target_quat.x();
          pose_pub.pose.orientation.y = target_quat.y();
          pose_pub.pose.orientation.z = target_quat.z();
          pose_pub.pose.orientation.w = target_quat.w();
          target_pose_pub_.publish(pose_pub);
        }
        ready2_publish = true;
        return;
      }

      else{
        it_bef_height_filter = it_bef_height_filter + 1;

        if (it_bef_height_filter <5) return;
      }
    }

    // FUNCTION TO TRANSFORM THE SCENE FLOOR PERP. TO THE Z SENSOR AXIS
    if (orientation_req_PC || !user_def_plane){
      pcl::copyPointCloud(*input_scene, *original_scene);
      pcl::transformPointCloud (*input_scene, *input_scene, rot_matrix);
      Eigen::Matrix4f rotation_matrix4;
      rotation_matrix4 = rot_matrix.matrix();
      initial_guess = initial_guess* rot_matrix.matrix() ;

      if (initialized){
        pcl::transformPointCloud (*previous_target, *previous_target, rot_matrix);
      }

      if (debug_set_inclination){
        pcl::visualization::PCLVisualizer viewer16 ("Debug set inclination");
        viewer16.addCoordinateSystem (1.0);
        pcl::visualization::PointCloudColorHandlerCustom<PointType> green_color(input_scene, 0, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<PointType> red_color(original_scene, 255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<PointType> ambar_color(previous_target, 255, 255, 0);

        viewer16.addPointCloud<PointType> (input_scene, green_color, "input_scene1");
        viewer16.addPointCloud<PointType> (original_scene, red_color, "original_scene1");
        viewer16.addPointCloud<PointType> (previous_target, ambar_color, "previous_target1");
        while(!viewer16.wasStopped()) viewer16.spinOnce (1);
      }
    }

    // HEIGHT FILTER TO BE APPLIED TO REJECT HEIGHT OUTLIERS FROM SENSOR
    if (filter_range_) (heightFilter(input_scene, input_scene, min_range_, max_range_));

    // **
    // DETECTION PART: Registration the target & scene at different heights of the scene
    // **
    ready2_publish = false;
    float sc_min_height_point, sc_max_height_point;
    getMinAndMaxPoints(input_scene, sc_min_height_point, sc_max_height_point);

    float top_height = sc_max_height_point;
    float bottom_height = sc_max_height_point + layer_height;

    float min_loop_score = 1;

    while (bottom_height < sc_min_height_point){

      pcl::PointCloud<PointType>::Ptr height_filt_scene (new pcl::PointCloud<PointType> ());
      heightFilter(input_scene, height_filt_scene, top_height, bottom_height);

      if ((height_filt_scene->points.size()) > min_scene_point_size){

        pcl::PointCloud<PointType>::Ptr previous_target_aligned (new pcl::PointCloud<PointType> ());
        Eigen::Vector4f s_centroid;
        // Move target to align the centroid
        pcl::compute3DCentroid(*height_filt_scene, s_centroid);
        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        transform_2.translation() << s_centroid[0], s_centroid[1], s_centroid[2];

        pcl::transformPointCloud (*previous_target, *previous_target_aligned, transform_2);
        Eigen::Affine3f output_trans = Eigen::Affine3f::Identity();
        float reg_score;
        if (!initialized) targetOrientationEvaluation(height_filt_scene, s_centroid, reg_result, reg_score, output_trans);

        else {
          Eigen::Matrix4f i_guess = transform_2.matrix();
          registrationPC(best_target_reg_0, i_guess, height_filt_scene, height_reg_sc_th, 5.5 , reg_result, reg_score, output_trans);
        }
        // Correspondence evaluation between Scene & target

        if (debug_height){
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
          initialized = true;
          Eigen::Vector3f target_pos;
          Eigen::Quaternionf target_quat;

          if (orientation_req_PC){
            pcl::PointCloud<PointType>::Ptr sensor_reg_result (new pcl::PointCloud<PointType> ());
            pcl::transformPointCloud (*reg_result, *sensor_reg_result, anti_rot_matrix);
            std::cout << " OUtput_trans" << output_trans.matrix() << "anti_rot_matrix" << anti_rot_matrix.matrix() << std::endl;
            Eigen::Matrix4f anti_rot_matrix_4f;
            anti_rot_matrix_4f = anti_rot_matrix.matrix();
            Eigen::Matrix4f output_trans_4f;
            output_trans_4f = output_trans.matrix();
            initial_guess = anti_rot_matrix_4f * output_trans_4f;

            detectedTargetPositionAdapt(sensor_reg_result,target_pos, target_quat);
          }

          else detectedTargetPositionAdapt(reg_result, target_pos, target_quat);

          it_bef_height_filter = 0;
          return;
        }
      }

      top_height = top_height + incr_layer_height;
      bottom_height = bottom_height + incr_layer_height;
    }

    if (!initialized) (std::cout << "Initial target not found...  \n"  <<std:: endl);
    std::cout << "Minimum Loop score = " << min_loop_score << "" <<std:: endl;
    return;
  }
}
