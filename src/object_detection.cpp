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
    nodeHandle.param<std::string>("target_0", target0_filename, "vertices14_0.pcd");
    nodeHandle.param<std::string>("target_1", target1_filename, "vertices14_90.pcd");
    nodeHandle.param<std::string>("target_2", target2_filename, "vertices14_180.pcd");
    nodeHandle.param<std::string>("target_3", target3_filename, "vertices14_270.pcd");
    // DEBUG
    nodeHandle.param("debug_detected_object", debug_detected_object, false); 
    nodeHandle.param("debug_height", debug_height, false); 
    nodeHandle.param("debug_set_inclination", debug_set_inclination, false);
    nodeHandle.param("debug_max_min_height", debug_max_min_height, false);
    nodeHandle.param("debug_best_target", debug_best_target, false); 
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
  
    if (pcl::io::loadPCDFile (target2_filename, *target2_) < 0){
      std::cout << "Error loading target-2 cloud." << std::endl;
      return;
    }
    
    if (pcl::io::loadPCDFile (target3_filename, *target3_) < 0){
      std::cout << "Error loading target-3 cloud." << std::endl;
      return;
    }
    
    pcl::compute3DCentroid(*previous_target, m_centroid);      
    pcl::compute3DCentroid(*target1_, tar1_centroid);   
    pcl::compute3DCentroid(*target2_, tar2_centroid);
    pcl::compute3DCentroid(*target3_, tar3_centroid);
    
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
 
  // TODO rewrite this function
    void ObjectDetection::registrationPC(pcl::PointCloud<PointType>::ConstPtr points_with_normals_src, 
                                         pcl::PointCloud<PointType>::ConstPtr scene, float minimum_score_dif,
                                         float max_correspondence_dist, pcl::PointCloud<PointType>::Ptr& reg_result,
                                         float & reg_score ){
      // TODO check if need this to avoid random number due to the no measurements
      reg_score = 1.0;
      // TODO revise this function 
      pcl::IterativeClosestPointNonLinear<PointType, PointType> reg;
      reg.setTransformationEpsilon (1e-6); //1e-6
      reg.setMaxCorrespondenceDistance (max_correspondence_dist);  //5.5 good initial 0.1
      reg.setInputSource (points_with_normals_src);
      reg.setInputTarget (scene);
      Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
      pcl::PointCloud<PointType>::Ptr var_point_cloud (new pcl::PointCloud<PointType> ()); 
      pcl::copyPointCloud(*points_with_normals_src, *var_point_cloud); 
      reg.setMaximumIterations (2);
      reg_result = var_point_cloud ;
      float score;
      float prev_score = 0;

      for (int i = 0; i < 5; ++i) {
        // save cloud for visualization purpose
        var_point_cloud = reg_result;

        // Estimate
        reg.setInputSource (var_point_cloud);
        reg.align (*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;   

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
          reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
        
        prev = reg.getLastIncrementalTransformation ();
        score = reg.getFitnessScore();
        
        //std::cout << "ITERATION=" << i << "; SCORE=" << score << "\n" << std::endl;
        if (sqrt ((prev_score - score)*(prev_score - score)) < minimum_score_dif) {
          reg_score = score;
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
                                                      float & best_score){
      // TODO generate the four targets and get the transformation between each target & the scene 
      float score[4];
      pcl::PointCloud<PointType>::Ptr reg_result0 (new pcl::PointCloud<PointType> ()); 
      pcl::PointCloud<PointType>::Ptr reg_result1 (new pcl::PointCloud<PointType> ()); 
      pcl::PointCloud<PointType>::Ptr reg_result2 (new pcl::PointCloud<PointType> ()); 
      pcl::PointCloud<PointType>::Ptr reg_result3 (new pcl::PointCloud<PointType> ()); 
      pcl::PointCloud<PointType>::Ptr target_aligned (new pcl::PointCloud<PointType> ()); 
      Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
      // TARGET #1
      transform_2.translation() << s_centroid[0] - m_centroid[0],
                                   s_centroid[1] - m_centroid[1], 
                                   s_centroid[2] - m_centroid[2];
      pcl::transformPointCloud (*previous_target, *target_aligned, transform_2);  
      registrationPC(target_aligned, height_filt_scene, height_reg_sc_th, 5.5 , reg_result0, score[0]);
      pcl::copyPointCloud(*reg_result0, *best_target_reg);
      best_score = score[0];
      // TARGET #2
      transform_2.translation() << s_centroid[0] - tar1_centroid[0],
                                   s_centroid[1] - tar1_centroid[1], 
                                   s_centroid[2] - tar1_centroid[2];
      pcl::transformPointCloud (*target1_, *target_aligned, transform_2);
      registrationPC(target_aligned, height_filt_scene, height_reg_sc_th, 5.5 , reg_result1, score[1]);
      if (score[1] < best_score){
        pcl::copyPointCloud(*reg_result1, *best_target_reg);
        best_score = score[1];
      }
      // TARGET #3      
      transform_2.translation() << s_centroid[0] - tar2_centroid[0], 
                                   s_centroid[1] - tar2_centroid[1],
                                   s_centroid[2] - tar2_centroid[2];
      pcl::transformPointCloud (*target2_, *target_aligned, transform_2);
      registrationPC(target_aligned, height_filt_scene, height_reg_sc_th, 5.5 , reg_result2, score[2]);
      if (score[2] < best_score){
        pcl::copyPointCloud(*reg_result2, *best_target_reg);
        best_score = score[2];
      }
      // TARGET #4     
      transform_2.translation() << s_centroid[0] - tar3_centroid[0],
                                   s_centroid[1] - tar3_centroid[1],
                                   s_centroid[2] - tar3_centroid[2];
      pcl::transformPointCloud (*target3_, *target_aligned, transform_2);
      registrationPC(target_aligned, height_filt_scene, height_reg_sc_th, 5.5 , reg_result3, score[3]);
      if (score[3] < best_score){
        pcl::copyPointCloud(*reg_result3, *best_target_reg);
        best_score = score[3];
      }
                
      if (debug_best_target){
        std::cout << "SCORE 0 Degrees   =  " << score[0] << "--"<< "\n" <<std:: endl; 
        std::cout << "SCORE 90 Degrees  =  " << score[1] << "--"<<"\n" <<std:: endl; 
        std::cout << "SCORE 180 Degrees =  " << score[2] << "--"<<"\n" <<std:: endl; 
        std::cout << "SCORE 270 Degrees =  " << score[3] <<"--"<< "\n" <<std:: endl; 
        std::cout << "AND THE BEST IS   =  " << best_score << "\n" << std:: endl;   

        pcl::visualization::PointCloudColorHandlerCustom<PointType> red_color(height_filt_scene, 255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<PointType> green_color(best_target_reg, 0, 255, 0);

        pcl::visualization::PCLVisualizer viewer5 ("TARGET AT 0 DEG");        
        viewer5.addPointCloud<PointType> (height_filt_scene, red_color, "Scene filtered");        
        viewer5.addPointCloud<PointType> (best_target_reg, green_color, "Target best ");
        
        pcl::visualization::PCLVisualizer viewer6 ("TARGET AT 90 DEG");        
        viewer6.addPointCloud<PointType> (height_filt_scene, red_color, "Scene filtered");
        viewer6.addPointCloud<PointType> (reg_result1, green_color, "Target 0 ");
        
        pcl::visualization::PCLVisualizer viewer7 ("TARGET AT 180 DEG ");        
        viewer7.addPointCloud<PointType> (height_filt_scene, red_color, "Scene filtered");
        viewer7.addPointCloud<PointType> (reg_result2,  green_color, "target Registered ");
        
        pcl::visualization::PCLVisualizer viewer8 ("TARGET AT 270 DEG");        
        viewer8.addPointCloud<PointType> (height_filt_scene, red_color, "Scene filtered");
        viewer8.addPointCloud<PointType> (reg_result3,  green_color, "target Registered ");  

        pcl::visualization::PCLVisualizer viewer9 ("BEST TARGET");
        viewer9.addPointCloud<PointType> (height_filt_scene, red_color, "Scene filtered");
        viewer9.addPointCloud<PointType> (best_target_reg, green_color, "Target best ");    

        while(!viewer5.wasStopped()) viewer5.spinOnce (1);
        while(!viewer6.wasStopped()) viewer6.spinOnce (1);
        while(!viewer7.wasStopped()) viewer7.spinOnce (1);
        while(!viewer8.wasStopped()) viewer8.spinOnce (1);
        while(!viewer9.wasStopped()) viewer9.spinOnce (1);
      }
      
    }
    // TODO MODIFY THE NAME FROM detectedTargetPositionAdapt TO TARGET FOUND LOCATION TRANSFORMATION OR SOMTHING LIKE THIS
    void ObjectDetection::detectedTargetPositionAdapt(pcl::PointCloud<PointType>::ConstPtr object_detected,
                                          pcl::PointCloud<PointType>::ConstPtr scene, 
                                          const sensor_msgs::PointCloud2::ConstPtr& in_cloud,
                                          Eigen::Vector3f & target_pos, Eigen::Quaternionf & target_quat ){          
      // 1. GET THE POSITION AND THE LOCATION OF THE TARGET FOUND
      float target_min_height_point, target_max_height_point;
      PointType target_position_OBB , target_min_point_OBB, target_max_point_OBB;
      Eigen::Matrix3f target_rotational_matrix_OBB;
      Eigen::Vector3f major_vector, middle_vector, minor_vector;
      Eigen::Vector3f mass_center;
      std::vector <float> moment_of_inertia;
      float major_value, middle_value, minor_value;      
      pcl::MomentOfInertiaEstimation <PointType> feature_extractor;
      // 
      // Compute  
      // 
      feature_extractor.setInputCloud (object_detected);
      feature_extractor.compute ();
      feature_extractor.getMomentOfInertia (moment_of_inertia);
      feature_extractor.getOBB (target_min_point_OBB, target_max_point_OBB, target_position_OBB, target_rotational_matrix_OBB);
      feature_extractor.getEigenValues (major_value, middle_value, minor_value);
      feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
      feature_extractor.getMassCenter (mass_center);
      Eigen::Vector3f target_position (target_position_OBB.x, target_position_OBB.y, target_position_OBB.z);

      // 2. TRANSFORM TO THE REQUIRED OUTPUT ORIENTATION (Z AXIS DOWNWARDS, Y ALONG THE TARGET)
      Eigen::Affine3f z_down_rot_matrix; 
      float angle = M_PI;
      z_down_rot_matrix (0,0)= cos(angle)+ middle_vector(0)*middle_vector(0)* (1 - cos(angle)); 
      z_down_rot_matrix (0,1)= middle_vector(0)* middle_vector(1)*(1-cos(angle))-middle_vector(2)*sin(angle); 
      z_down_rot_matrix (0,2)= middle_vector(0)*middle_vector(2)*(1-cos(angle))+ middle_vector(1)*sin(angle);

      z_down_rot_matrix (1,0)= middle_vector(1)*middle_vector(0)*(1-cos(angle))+middle_vector(2)*sin(angle); 
      z_down_rot_matrix (1,1)= cos(angle)+ middle_vector(1)*middle_vector(1)*(1-cos(angle)); 
      z_down_rot_matrix (1,2)= middle_vector(1)*middle_vector(2)*(1-cos(angle))-middle_vector(0)*sin(angle);

      z_down_rot_matrix (2,0)= middle_vector(2)*middle_vector(0)*(1-cos(angle))-middle_vector(1)*sin(angle); 
      z_down_rot_matrix (2,1)= middle_vector(2)*middle_vector(1)*(1-cos(angle))+middle_vector(0)*sin(angle); 
      z_down_rot_matrix (2,2)= cos(angle)+middle_vector(2)*middle_vector(2)*(1-cos(angle));
      
      Eigen::Vector3f rot_major_vector, rot_middle_vector, rot_minor_vector;
      rot_major_vector = ( z_down_rot_matrix) * major_vector;
      rot_middle_vector =  (z_down_rot_matrix) *  middle_vector;
      rot_minor_vector =  (z_down_rot_matrix) * minor_vector; 
      
      // 3. PREPARE DATA FOR NEXT ITERATION CONVERT FROM ROTATIONAL MATRIX TO QUATERNION

      //TODO modify initial guess to target need initialize initial_guess globally like identify
      // USED THE SET THE INITIAL GUESS
      // TODO MODIFY IT 
      Eigen::Matrix4f target_transform = Eigen::Matrix4f::Identity();
      target_transform (0,0)= target_rotational_matrix_OBB(0,0); 
      target_transform (0,1)= target_rotational_matrix_OBB(0,1); 
      target_transform (0,2)= target_rotational_matrix_OBB(0,2);
      target_transform (1,0)= target_rotational_matrix_OBB(1,0); 
      target_transform (1,1)= target_rotational_matrix_OBB(1,1); 
      target_transform (1,2)= target_rotational_matrix_OBB(1,2);
      target_transform (2,0)= target_rotational_matrix_OBB(2,0); 
      target_transform (2,1)= target_rotational_matrix_OBB(2,1); 
      target_transform (2,2)= target_rotational_matrix_OBB(2,2);
      target_transform (0,3)= target_position_OBB.x;
      target_transform (1,3)= target_position_OBB.y;
      target_transform (2,3)= target_position_OBB.z;
      initial_guess = target_transform; 
      pcl::copyPointCloud(*object_detected, *previous_target);

      //4. PREPARE DATA TO PUBLISH (CONVERSION ROTATIONAL MATRIX TO QUATERNION FOR POSE PUB)
      Eigen::Quaternionf target_quat2 (target_rotational_matrix_OBB);
      target_quat = target_quat2;
      target_pos = mass_center;
  
      if (debug_detected_object){
        pcl::visualization::PCLVisualizer viewer5 ("Final object detection");
        pcl::visualization::PointCloudColorHandlerCustom<PointType> green_color(reg_result, 0, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<PointType> ambar_color(object_detected, 255, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<PointType> red_color(scene, 255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<PointType> blue_color(original_scene, 0, 0, 255);
        viewer5.addCoordinateSystem (1.0);
        viewer5.addPointCloud<PointType> (reg_result, green_color, "target Registered ");          
        viewer5.addPointCloud<PointType> (object_detected, ambar_color, "target Registereds ");        
        viewer5.addPointCloud<PointType> (scene, red_color, "Scene filtered");         
        viewer5.addPointCloud<PointType> (original_scene, blue_color, "original scene");
        viewer5.addCube (target_position, target_quat, 
                        target_max_point_OBB.x - target_min_point_OBB.x, target_max_point_OBB.y - target_min_point_OBB.y,
                        target_max_point_OBB.z - target_min_point_OBB.z, "target OBB");

        pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
        pcl::PointXYZ x_axis (rot_major_vector (0) + mass_center (0),
                              rot_major_vector (1) + mass_center (1),
                              rot_major_vector (2) + mass_center (2));
        pcl::PointXYZ y_axis (rot_middle_vector (0) + mass_center (0), 
                              rot_middle_vector (1) + mass_center (1), 
                              rot_middle_vector (2) + mass_center (2));
        pcl::PointXYZ z_axis (rot_minor_vector (0) + mass_center (0), 
                              rot_minor_vector (1) + mass_center (1), 
                              rot_minor_vector (2) + mass_center (2));
        viewer5.addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
        viewer5.addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
        viewer5.addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

        std::cout << "Initial guess = " << initial_guess << "\n" <<std:: endl; 
        std::cout << "Initial guess pose  = " << initial_guess(0,3) << ", "<< initial_guess(1,3) << ", "
                                              << initial_guess(2,3) << ", " <<std:: endl;
        std::cout << "**Target found **"  << "\n" <<std:: endl;
        while(!viewer5.wasStopped())viewer5.spinOnce (1);
    }
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
      registrationPC(previous_target, input_scene, reg_sc_th, 5.5, reg_result, reg_initial_score);
       
      if (reg_initial_score < threshold_score_){ 
        //std::cout << "MATHCES  FOUND, Score = " << reg_initial_score << "\n" <<std:: endl;       
        pcl::PointCloud<PointType>::Ptr sensor_reg_result (new pcl::PointCloud<PointType> ());
   
        Eigen::Vector3f target_pos;
        Eigen::Quaternionf target_quat;
        detectedTargetPositionAdapt(reg_result,input_scene, in_cloud, target_pos, target_quat);

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

          // POSE 
          geometry_msgs::PoseStamped pose_pub;

          pose_pub.header = in_cloud->header;
          pose_pub.pose.position.x = target_pos(0,3);
          pose_pub.pose.position.y = target_pos(1,3);
          pose_pub.pose.position.z = target_pos(2,3);
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
    pcl::compute3DCentroid(*previous_target, m_centroid);

    float top_height = sc_max_height_point;
    float bottom_height = sc_max_height_point + layer_height;
    
    float min_loop_score = 1;

    while (bottom_height < sc_min_height_point){
      /*std::cout << "***top_height= "<< top_height << ";  Bottom_height= " << bottom_height <<
                  "; sc_min_height_point="<< sc_min_height_point <<"; sc_max_height_point="<< 
                  sc_max_height_point << "\n"<< std:: endl;*/

      pcl::PointCloud<PointType>::Ptr height_filt_scene (new pcl::PointCloud<PointType> ());     
      heightFilter(input_scene, height_filt_scene, top_height, bottom_height);     

      if ((height_filt_scene->points.size()) > min_scene_point_size){
         
        pcl::PointCloud<PointType>::Ptr previous_target_aligned (new pcl::PointCloud<PointType> ());
        Eigen::Vector4f s_centroid;    
        // Move target to align the centroid
        pcl::compute3DCentroid(*height_filt_scene, s_centroid);
        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        transform_2.translation() << s_centroid[0] - m_centroid[0], s_centroid[1] - m_centroid[1], s_centroid[2] - m_centroid[2];
        pcl::transformPointCloud (*previous_target, *previous_target_aligned, transform_2);       
        float reg_score;
        if (!initialized) targetOrientationEvaluation(height_filt_scene, s_centroid, reg_result, reg_score);                                 
           
        else registrationPC(previous_target_aligned, height_filt_scene, height_reg_sc_th, 5.5 , reg_result, reg_score);
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
            detectedTargetPositionAdapt(sensor_reg_result,input_scene, in_cloud,target_pos, target_quat); 
          }

          else detectedTargetPositionAdapt(reg_result,input_scene, in_cloud, target_pos, target_quat);  

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
