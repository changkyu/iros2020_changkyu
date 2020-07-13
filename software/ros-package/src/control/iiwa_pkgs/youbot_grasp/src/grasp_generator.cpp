#include <youbot_grasp/grasp_generator.h>

/*constructor*/
  std::vector<fcl::Vec3f> GraspGenerator::vertices_cube_;
  std::vector<fcl::Triangle> GraspGenerator::triangles_cube_;
  std::vector<fcl::Vec3f> GraspGenerator::vertices_floor_;
  std::vector<fcl::Triangle> GraspGenerator::triangles_floor_;
  std::vector<fcl::Vec3f> GraspGenerator::vertices_collision_gripper_;
  std::vector<fcl::Triangle> GraspGenerator::triangles_collision_gripper_;
  std::vector<fcl::Vec3f> GraspGenerator::vertices_base_;
  std::vector<fcl::Triangle> GraspGenerator::triangles_base_;
  std::vector<fcl::Vec3f> GraspGenerator::vertices_opti_cube_;
  std::vector<fcl::Triangle> GraspGenerator::triangles_opti_cube_;
  std::vector<fcl::Vec3f> GraspGenerator::vertices_standard_gripper_;
  std::vector<fcl::Triangle> GraspGenerator::triangles_standard_gripper_;
  std::vector<fcl::Vec3f>  GraspGenerator::vertices_arm0_;
  std::vector<fcl::Triangle> GraspGenerator::triangles_arm0_;
  std::vector<fcl::Vec3f> GraspGenerator::vertices_arm1_;
  std::vector<fcl::Triangle> GraspGenerator::triangles_arm1_;
  std::vector<fcl::Vec3f> GraspGenerator::vertices_arm2_;
  std::vector<fcl::Triangle> GraspGenerator::triangles_arm2_;
  std::vector<fcl::Vec3f> GraspGenerator::vertices_arm3_;
  std::vector<fcl::Triangle> GraspGenerator::triangles_arm3_;
  std::vector<fcl::Vec3f> GraspGenerator::vertices_arm4_;
  std::vector<fcl::Triangle> GraspGenerator::triangles_arm4_;
  std::vector<fcl::Vec3f> GraspGenerator::vertices_arm5_new_;
  std::vector<fcl::Triangle> GraspGenerator::triangles_arm5_new_;



GraspGenerator::GraspGenerator(float R, ros::NodeHandle &node)
{
  radius = R;    
  node_ = &node;

  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/base_frame_convex.obj").c_str(), GraspGenerator::vertices_base_, GraspGenerator::triangles_base_);
  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/collision_gripper.obj").c_str(), GraspGenerator::vertices_collision_gripper_, GraspGenerator::triangles_collision_gripper_);
  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/cube.obj").c_str(), GraspGenerator::vertices_cube_, GraspGenerator::triangles_cube_);
  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/floor.obj").c_str(), GraspGenerator::vertices_floor_, GraspGenerator::triangles_floor_);
   loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/opti_cube.obj").c_str(), GraspGenerator::vertices_opti_cube_, GraspGenerator::triangles_opti_cube_);
 loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/standard_gripper.obj").c_str(), GraspGenerator::vertices_standard_gripper_, GraspGenerator::triangles_standard_gripper_);

 loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/arm0_convex.obj").c_str(), GraspGenerator::vertices_arm0_, GraspGenerator::triangles_arm0_);
  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/arm1_convex.obj").c_str(), GraspGenerator::vertices_arm1_, GraspGenerator::triangles_arm1_);
  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/arm2_convex.obj").c_str(), GraspGenerator::vertices_arm2_, GraspGenerator::triangles_arm2_);
  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/arm3_convex.obj").c_str(), GraspGenerator::vertices_arm3_, GraspGenerator::triangles_arm3_);
  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/arm4_convex.obj").c_str(), GraspGenerator::vertices_arm4_, GraspGenerator::triangles_arm4_);

  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/arm5_new_convex.obj").c_str(), GraspGenerator::vertices_arm5_new_, GraspGenerator::triangles_arm5_new_);
}

/*destructor*/
GraspGenerator::~GraspGenerator()
{

}

void GraspGenerator::loadFile(){
  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/base_frame_convex.obj").c_str(), GraspGenerator::vertices_base_, GraspGenerator::triangles_base_);
  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/collision_gripper.obj").c_str(), GraspGenerator::vertices_collision_gripper_, GraspGenerator::triangles_collision_gripper_);
  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/cube.obj").c_str(), GraspGenerator::vertices_cube_, GraspGenerator::triangles_cube_);
  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/floor.obj").c_str(), GraspGenerator::vertices_floor_, GraspGenerator::triangles_floor_);
  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/standard_gripper.obj").c_str(), GraspGenerator::vertices_standard_gripper_, GraspGenerator::triangles_standard_gripper_);

  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/opti_cube.obj").c_str(), GraspGenerator::vertices_opti_cube_, GraspGenerator::triangles_opti_cube_);

  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/arm0_convex.obj").c_str(), GraspGenerator::vertices_arm0_, GraspGenerator::triangles_arm0_);
  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/arm1_convex.obj").c_str(), GraspGenerator::vertices_arm1_, GraspGenerator::triangles_arm1_);
  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/arm2_convex.obj").c_str(), GraspGenerator::vertices_arm2_, GraspGenerator::triangles_arm2_);
  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/arm3_convex.obj").c_str(), GraspGenerator::vertices_arm3_, GraspGenerator::triangles_arm3_);
  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/arm4_convex.obj").c_str(), GraspGenerator::vertices_arm4_, GraspGenerator::triangles_arm4_);

  loadOBJFile((ros::package::getPath("youbot_grasp")+"/models/arm5_new_convex.obj").c_str(), GraspGenerator::vertices_arm5_new_, GraspGenerator::triangles_arm5_new_);
}

/*  examine the candidate grasps, if they are not the suitable points for the suction cup to act on, they will be eliminated.
    The length of the cross section between the mesh and the circle reepresenting the cup around the candidate point is compared with the 
    circumference of the suction cup. If the ratio is within a certain range, the candidate grasp point is deemed suitable. 

    object_file_name, filename of the target STL file
    sample_point,  the sampled points from approach_dir_generator.cpp
    sample_normal, the unit normal direction of each sample point
*/
int GraspGenerator::generateGrasp(std::string object_file_name, std::vector<geometry_msgs::Point>& sample_point_not , std::vector<geometry_msgs::Point>& sample_normal_not, std::list<plannedGrasp*>& grasp_list, int gripper_type, youbot_grasp::PlanningScene * planning_scene, geometry_msgs::Pose target_pose, std::list<sampledPoint*> sampledPoint_list)
{
  MyMesh m, res;                
  float length;                               // length of the intersection between the ball and the mesh
  int grasp_num = 0;
  ros::Publisher fly_gripper_pub = node_->advertise<geometry_msgs::Pose>("/vrep/fly_gripper", 1000);
  double palm_fallback_scale[] = {0.09, 0.08, 0.07, 0.06, 0.05, 0.04, 0.03, 0.02, 0.01}; // unit: meter
  grasp_list.clear();
  std::string gripper_filename;
  if(gripper_type == 2){
    gripper_filename = "collision_gripper";
  }else if(gripper_type == 3){
    gripper_filename = "standard_gripper";
  }

    geometry_msgs::Pose gripper_pose;
    geometry_msgs::Pose pre_gripper_pose;
    geometry_msgs::Pose vrep_gripper_pose;                     //used for publish debug gripper pose messege
    std::list<sampledPoint*>::iterator q;
    std::vector<geometry_msgs::Point> sample_point;
    std::vector<geometry_msgs::Point> sample_normal;

    double biggestNearToRight = 0;
    for(q = sampledPoint_list.begin(); q != sampledPoint_list.end(); q ++){
      sample_point.push_back((*q)->sample_point_);
      sample_normal.push_back((*q)->sample_normal_);      //in object local frame
    }


    for(int i = 0;i < sample_point.size();++i)
    {
      biggestNearToRight = 0;
      #ifdef DEBUG_
      ROS_INFO_STREAM("Checking "<<i+1<<"sample point");
      #endif

      //eliminate sample_point facing floor, avoid further collision checking
      // if(sample_normal[i].z != 1){
      //   continue;
      // }

      for(int j = 0;j < 1;++j)  // every 30 degrees rotation of the hand
      {
        #ifdef DEBUG_
        ROS_INFO_STREAM("checking "<<j<<"th rotations");
        #endif
        tf::Quaternion palm_obj_q;
        tf::Matrix3x3 palm_obj_m;
        // handle relative rotations
        palm_obj_m = this->get_cp_obj_quaternion(sample_point[i], sample_normal[i], M_PI/2 /*M_PI/24 *j*/);  
      
        tf::Quaternion obj_q(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
        tf::Matrix3x3 obj_m(obj_q);
        tf::Matrix3x3 palm_m = obj_m * palm_obj_m;
        tf::Quaternion palm_q; //= obj_q * palm_obj_q
        palm_m.getRotation(palm_q);
        tf::Matrix3x3 test_m(palm_q);
        //tf::Matrix3x3 palm_obj_m(palm_obj_q);
        tf::Vector3 palm_obj_x_axis = palm_obj_m.getColumn(0);
        tf::Vector3 obj_y_axis(0, 1, 0);
        double nearToRight = fabs(palm_obj_x_axis.dot(obj_y_axis));
        if(nearToRight < biggestNearToRight){
          continue; 
        }else{
          biggestNearToRight = nearToRight;
        }


        int k = 0;
        for(k = 0; k < 9;k++){  
          #ifdef DEBUG_
          ROS_INFO_STREAM("checking palm distance :" << palm_fallback_scale[k]);
          #endif
          fcl::Vec3f fcl_gripper_v;
          fcl::Matrix3f fcl_gripper_m;
          fcl::Vec3f fcl_target_v;
          fcl::Matrix3f fcl_target_m;
          fcl::Vec3f fcl_pre_gripper_v;
          fcl::Matrix3f fcl_pre_gripper_m;
          bool config_ok = true; 
          //std::vector<tf::Vector3> collision_points;     
          
          geoPoseToFclPose(target_pose, fcl_target_v, fcl_target_m);   // pose for the target grasping object
          double gripper_x_relative_to_obj = sample_point[i].x + palm_fallback_scale[k]*sample_normal[i].x;
          double gripper_y_relative_to_obj = sample_point[i].y + palm_fallback_scale[k]*sample_normal[i].y;
          double gripper_z_relative_to_obj = sample_point[i].z + palm_fallback_scale[k]*sample_normal[i].z;
          tf::Vector3 gripper_relative_to_obj(gripper_x_relative_to_obj, gripper_y_relative_to_obj, gripper_z_relative_to_obj);
          double gripper_x_relative_to_obj_in_global_frame = obj_m.getRow(0).dot(gripper_relative_to_obj);
          double gripper_y_relative_to_obj_in_global_frame = obj_m.getRow(1).dot(gripper_relative_to_obj);
          double gripper_z_relative_to_obj_in_global_frame = obj_m.getRow(2).dot(gripper_relative_to_obj);
          
          double pre_gripper_x_relative_to_obj = (palm_fallback_scale[0] - palm_fallback_scale[k]) * sample_normal[i].x;
          double pre_gripper_y_relative_to_obj = (palm_fallback_scale[0] - palm_fallback_scale[k]) * sample_normal[i].y;
          double pre_gripper_z_relative_to_obj = (palm_fallback_scale[0] - palm_fallback_scale[k]) * sample_normal[i].z;
          tf::Vector3 pre_gripper_relative_to_obj(pre_gripper_x_relative_to_obj, pre_gripper_y_relative_to_obj, pre_gripper_z_relative_to_obj);
          double pre_gripper_x_relative_to_obj_in_global_frame = obj_m.getRow(0).dot(pre_gripper_relative_to_obj);
          double pre_gripper_y_relative_to_obj_in_global_frame = obj_m.getRow(1).dot(pre_gripper_relative_to_obj);
          double pre_gripper_z_relative_to_obj_in_global_frame = obj_m.getRow(2).dot(pre_gripper_relative_to_obj);

          gripper_pose.position.x = target_pose.position.x + gripper_x_relative_to_obj_in_global_frame;
          gripper_pose.position.y = target_pose.position.y + gripper_y_relative_to_obj_in_global_frame;
          gripper_pose.position.z = target_pose.position.z + gripper_z_relative_to_obj_in_global_frame;

          gripper_pose.orientation.x = palm_q.x();
          gripper_pose.orientation.y = palm_q.y();
          gripper_pose.orientation.z = palm_q.z();
          gripper_pose.orientation.w = palm_q.w();
          pre_gripper_pose.position.x = gripper_pose.position.x + pre_gripper_x_relative_to_obj_in_global_frame;
          pre_gripper_pose.position.y = gripper_pose.position.y + pre_gripper_y_relative_to_obj_in_global_frame;
          pre_gripper_pose.position.z = gripper_pose.position.z + pre_gripper_z_relative_to_obj_in_global_frame;
          pre_gripper_pose.orientation.x = gripper_pose.orientation.x;
          pre_gripper_pose.orientation.y = gripper_pose.orientation.y;
          pre_gripper_pose.orientation.z = gripper_pose.orientation.z;
          pre_gripper_pose.orientation.w = gripper_pose.orientation.w;

          geoPoseToFclPose(gripper_pose, fcl_gripper_v, fcl_gripper_m); 
          geoPoseToFclPose(pre_gripper_pose, fcl_pre_gripper_v, fcl_pre_gripper_m); 
          #ifdef DEBUG_
          ROS_INFO_STREAM("target_pose:"<<target_pose.position.x<<","<<target_pose.position.y<<","<<target_pose.position.z);
          ROS_INFO_STREAM("gripper_pose:"<<gripper_pose.position.x<<","<<gripper_pose.position.y<<","<<gripper_pose.position.z);
          #endif
          vrep_gripper_pose = gripper_pose;
        //  vrep_gripper_pose.position.z -= 0.0285;
          //fly_gripper_pub.publish(vrep_gripper_pose);
          //ros::Duration(0.3).sleep(); 
            // pose for the target grasping object
          std::vector<fcl::Vec3f> fcl_scene_object_v_list;
          std::vector<fcl::Matrix3f> fcl_scene_object_m_list;
          fcl_scene_object_v_list.resize(planning_scene->scene_object_pose.size());
          fcl_scene_object_m_list.resize(planning_scene->scene_object_pose.size());
          for(int p = 0;p < planning_scene->scene_object_pose.size();p++){
            geoPoseToFclPose(planning_scene->scene_object_pose[p], fcl_scene_object_v_list[p], fcl_scene_object_m_list[p]);
          }
          // while( getchar() != '\n' );
          //check collision between gripper with target object
          if(checkCollision(gripper_filename, object_file_name, fcl_gripper_v, fcl_target_v, fcl_gripper_m, fcl_target_m) || checkCollision(gripper_filename, object_file_name, fcl_pre_gripper_v, fcl_target_v, fcl_pre_gripper_m, fcl_target_m)){
            if(k <= 5){
              config_ok = false;
            }else{
              config_ok = true;
            }
          }else{
            config_ok = true; 
          }
          if(config_ok){
            if(checkCollision(gripper_filename, "floor", fcl_gripper_v, fcl_scene_object_v_list[planning_scene->scene_object_pose.size()-1], fcl_gripper_m, fcl_scene_object_m_list[planning_scene->scene_object_pose.size()-1]) || checkCollision(gripper_filename, "floor", fcl_pre_gripper_v, fcl_scene_object_v_list[planning_scene->scene_object_pose.size()-1], fcl_pre_gripper_m, fcl_scene_object_m_list[planning_scene->scene_object_pose.size()-1])){
              config_ok = false;
              break;
            }
            for(int q = 0;q < planning_scene->scene_object_pose.size()-1;q++){
              if(checkCollision(gripper_filename, object_file_name, fcl_gripper_v, fcl_scene_object_v_list[q], fcl_gripper_m, fcl_scene_object_m_list[q]) || checkCollision(gripper_filename, object_file_name, fcl_pre_gripper_v, fcl_scene_object_v_list[q], fcl_pre_gripper_m, fcl_scene_object_m_list[q])){
                
                config_ok = false;
                break;
              }
            }

          }
          if(!config_ok){
            break;
          }
           

           
        } 
        if(k > 4){
             //while( getchar() != '\n' );
            ROS_INFO_STREAM("---------------------------------------------");
            ROS_INFO_STREAM("adding grasp candidate to grasp list-----------");

            double global_sample_point_x, global_sample_point_y, global_sample_point_z, global_sample_normal_x, global_sample_normal_y, global_sample_normal_z;
            ROS_INFO_STREAM("grasp normal:"<<sample_normal[i].x<<","<<sample_normal[i].y<<","<<sample_normal[i].z);
            ROS_INFO_STREAM("grasp point:"<<sample_point[i].x<<","<<sample_point[i].y<<","<<sample_point[i].z);
            turnFromLocalToGlobal(obj_m, sample_point[i].x, sample_point[i].y, sample_point[i].z, global_sample_point_x, global_sample_point_y, global_sample_point_z);
            tf::Vector3 point(global_sample_point_x, global_sample_point_y, global_sample_point_z);
            turnFromLocalToGlobal(obj_m, sample_normal[i].x, sample_normal[i].y, sample_normal[i].z, global_sample_normal_x, global_sample_normal_y, global_sample_normal_z);

            tf::Vector3 dir(global_sample_normal_x, global_sample_normal_y, global_sample_normal_z); 
            cartesianGraspDirection in;
            in.set_point(point);
            in.set_dir(dir);
            ROS_INFO_STREAM("target_pose xaxis:"<<obj_m.getColumn(0).getX()<<","<<obj_m.getColumn(0).getY()<<","<<obj_m.getColumn(0).getZ());
            ROS_INFO_STREAM("target_pose yaxis:"<<obj_m.getColumn(1).getX()<<","<<obj_m.getColumn(1).getY()<<","<<obj_m.getColumn(1).getZ());
            ROS_INFO_STREAM("target_pose zaxis:"<<obj_m.getColumn(2).getX()<<","<<obj_m.getColumn(2).getY()<<","<<obj_m.getColumn(2).getZ());

            ROS_INFO_STREAM("global grasp normal:"<<global_sample_normal_x<<","<<global_sample_normal_y<<","<<global_sample_normal_z);
            ROS_INFO_STREAM("global grasp point:"<<global_sample_point_x<<","<<global_sample_point_y<<","<<global_sample_point_z);
            double turn_back_in_global_frame_x, turn_back_in_global_frame_y, turn_back_in_global_frame_z;
            turnFromLocalToGlobal(obj_m, 0.01 * sample_normal[i].x, 0.01 * sample_normal[i].y, 0.01 * sample_normal[i].z, turn_back_in_global_frame_x, turn_back_in_global_frame_y, turn_back_in_global_frame_z);
            // gripper_pose.position.x = gripper_pose.position.x + turn_back_in_global_frame_x;
            // gripper_pose.position.y = gripper_pose.position.y + turn_back_in_global_frame_y;
            // gripper_pose.position.z = gripper_pose.position.z + turn_back_in_global_frame_z;
            plannedGrasp * planned_grasp = new plannedGrasp(in,  M_PI/2/*M_PI/24*j*/, palm_fallback_scale[k-1], gripper_pose, pre_gripper_pose);
            grasp_list.push_back(planned_grasp);
            grasp_num ++;
        }
        
        
      } // end of 12 rotation degrees
      if(grasp_num > 50){
                    
        //fly_gripper_pub.publish(grasp_list.front()->getGraspPose());
        break;
      }
    } // end of all the sample points   

  ROS_INFO_STREAM("return grasp num:"<<grasp_num);
  return grasp_num;
    	
}

void GraspGenerator::turnFromLocalToGlobal(tf::Matrix3x3 local_to_global_m, double local_x, double local_y, double local_z, double& global_x, double& global_y, double& global_z){
            tf::Vector3 local_vector(local_x, local_y, local_z);
            global_x = local_to_global_m.getRow(0).dot(local_vector);
            global_y = local_to_global_m.getRow(1).dot(local_vector);
            global_z = local_to_global_m.getRow(2).dot(local_vector);

}

void GraspGenerator::setRadius(float r)
{
  radius = r;
}

//generate all the collision points on all three fingers, all the collision points' coordinates are in the object frame.
// palm_obj_tf is the transform of the palm frame relative to the object frame, get all the frame of the collision points relative to the object frame from it.
// at last get all the collision points' coordinates relative to the object frame, then check the collision 
void GraspGenerator::generateCollisionPoints(tf::Transform palm_obj_tf, std::vector<tf::Vector3>& collision_points, int gripper_type, double palm_fallback = 0.0)
{  
  // if(gripper_type == 1)  // for the 3-finger gripper case
  // {  

  //   int finger_tilt_angle[9] = {40,0,10,40,0,10,40,0,10};
  //   //first finger, 
  //   tf::Transform finger_1_1_tf, finger_1_2_tf, finger_1_3_tf, finger_2_1_tf, finger_2_2_tf, finger_2_3_tf, finger_3_1_tf, finger_3_2_tf, finger_3_3_tf;
  //   tf::Matrix3x3 temp;
  //   double rotation_angle;
  
  //   //first finger 1 knot relative to palm frame
  //   finger_1_1_tf.setOrigin(tf::Vector3());
  //   rotation_angle = -(90-finger_tilt_angle[0])/180*M_PI;
  //   temp.setValue(cos(rotation_angle),-sin(rotation_angle),0,
  //               sin(rotation_angle),cos(rotation_angle),0,
  //               0,0,1);
  //   finger_1_1_tf.setBasis(temp);

  //   //first finger 2 knot relative to knot 1
  //   finger_1_2_tf.setOrigin(tf::Vector3(0.0389, 0, 0));
  //   rotation_angle = finger_tilt_angle[1]/180*M_PI;
  //   temp.setValue(cos(rotation_angle),-sin(rotation_angle),0,
  //               sin(rotation_angle),cos(rotation_angle),0,
  //               0,0,1);
  //   finger_1_2_tf.setBasis(temp);

  //   //first finger 3 knot relative to knot 2
  //   finger_1_3_tf.setOrigin(tf::Vector3(0.0389, 0, 0));
  //   rotation_angle = finger_tilt_angle[2]/180*M_PI;
  //   temp.setValue(cos(rotation_angle),-sin(rotation_angle),0,
  //               sin(rotation_angle),cos(rotation_angle),0,
  //               0,0,1);
  //   finger_1_3_tf.setBasis(temp);

  //   //second finger knot 1 relative to palm frame
  //   finger_2_1_tf.setOrigin(tf::Vector3());
  //   rotation_angle = -(90-finger_tilt_angle[3])/180*M_PI;
  //   temp.setValue(cos(rotation_angle),-sin(rotation_angle),0,
  //               sin(rotation_angle),cos(rotation_angle),0,
  //               0,0,1);
  //   finger_2_1_tf.setBasis(temp);

  //   //second finger knot 2 relative to knot 1
  //   finger_2_2_tf.setOrigin(tf::Vector3(0.0389, 0, 0));
  //   rotation_angle = finger_tilt_angle[4]/180*M_PI;
  //   temp.setValue(cos(rotation_angle),-sin(rotation_angle),0,
  //               sin(rotation_angle),cos(rotation_angle),0,
  //               0,0,1);
  //   finger_2_2_tf.setBasis(temp);

  //   //second finger knot 3 relative to knot 2
  //   finger_2_3_tf.setOrigin(tf::Vector3(0.0389, 0, 0));
  //   rotation_angle = finger_tilt_angle[5]/180*M_PI;
  //   temp.setValue(cos(rotation_angle),-sin(rotation_angle),0,
  //               sin(rotation_angle),cos(rotation_angle),0,
  //               0,0,1);
  //   finger_2_3_tf.setBasis(temp);

  //   //third finger knot 1 relative to the palm
  //   finger_3_1_tf.setOrigin(tf::Vector3());
  //   rotation_angle = -(90-finger_tilt_angle[6])/180*M_PI;
  //   temp.setValue(cos(rotation_angle), -sin(rotation_angle), 0,
  //               -sin(rotation_angle),-cos(rotation_angle), 0,
  //               0, 0, -1);
  //   finger_3_1_tf.setBasis(temp);

  //   //third finger knot 2 relative to knot 1
  //   finger_3_2_tf.setOrigin(tf::Vector3(0.0389,0,0));
  //   rotation_angle = finger_tilt_angle[7]/180*M_PI;
  //   temp.setValue(cos(rotation_angle),-sin(rotation_angle),0,
  //               sin(rotation_angle),cos(rotation_angle),0,
  //               0,0,1);
  //   finger_3_2_tf.setBasis(temp);

  //   //third finger knot 3 relative to knot 2
  //   finger_3_3_tf.setOrigin(tf::Vector3(0.0389, 0, 0));
  //   rotation_angle = finger_tilt_angle[8]/180*M_PI;
  //   temp.setValue(cos(rotation_angle),-sin(rotation_angle),0,
  //               sin(rotation_angle),cos(rotation_angle),0,
  //               0,0,1);
  //   finger_3_3_tf.setBasis(temp);

  //   tf::Vector3 touch_local(0.0194,0.011,0);//coordinate of the touch pad in local frame of a single knot
  //   tf::Transform finger_1_1_obj_tf, finger_1_2_obj_tf, finger_1_3_obj_tf, finger_2_1_obj_tf, finger_2_2_obj_tf, finger_2_3_obj_tf, 
  //         finger_3_1_obj_tf, finger_3_2_obj_tf, finger_3_3_obj_tf;
  //   finger_1_1_obj_tf = palm_obj_tf * finger_1_1_tf;
  //   finger_1_2_obj_tf = finger_1_1_obj_tf * finger_1_2_tf;
  //   finger_1_3_obj_tf = finger_1_2_obj_tf * finger_1_3_tf;

  //   finger_2_1_obj_tf = palm_obj_tf * finger_2_1_tf;
  //   finger_2_2_obj_tf = finger_2_1_obj_tf * finger_2_2_tf;
  //   finger_2_3_obj_tf = finger_2_2_obj_tf * finger_2_3_tf;

  //   finger_3_1_obj_tf = palm_obj_tf * finger_3_1_tf;
  //   finger_3_2_obj_tf = finger_3_1_obj_tf * finger_3_2_tf;
  //   finger_3_3_obj_tf = finger_3_2_obj_tf * finger_3_3_tf;

  //   tf::Vector3 touch_1_1, touch_1_2, touch_1_3, touch_2_1, touch_2_2, touch_2_3, touch_3_1, touch_3_2, touch_3_3;
  //   touch_1_1 = finger_1_1_obj_tf * touch_local;
  //   collision_points.push_back(touch_1_1);
  //   touch_1_2 = finger_1_2_obj_tf * touch_local;
  //   collision_points.push_back(touch_1_2);
  //   touch_1_3 = finger_1_3_obj_tf * touch_local;
  //   collision_points.push_back(touch_1_3);
  //   touch_2_1 = finger_2_1_obj_tf * touch_local;
  //   collision_points.push_back(touch_2_1);
  //   touch_2_2 = finger_2_2_obj_tf * touch_local;
  //   collision_points.push_back(touch_2_2);
  //   touch_2_3 = finger_2_3_obj_tf * touch_local;
  //   collision_points.push_back(touch_2_3);
  //   touch_3_1 = finger_3_1_obj_tf * touch_local;
  //   collision_points.push_back(touch_3_1);
  //   touch_3_2 = finger_3_2_obj_tf * touch_local;
  //   collision_points.push_back(touch_3_2);
  //   touch_3_3 = finger_3_3_obj_tf * touch_local;
  //   collision_points.push_back(touch_3_3);
  // }  // end for 3-finger gripper 
  if(gripper_type == 2)    // for m2 case
  { 
    
    //first finger, 
    tf::Transform finger_1_1_tf, finger_2_1_tf, finger_2_2_tf, finger_2_3_tf;
    tf::Matrix3x3 temp;
    double rotation_angle;
  
    //first finger 1 knot relative to palm frame
    finger_1_1_tf.setOrigin(tf::Vector3(0.0, 0.035, 0.0));
    temp.setValue(1,0,0,
                0,1,0,
                0,0,1);
    finger_1_1_tf.setBasis(temp);

    //second finger knot 1 relative to palm frame
    finger_2_1_tf.setOrigin(tf::Vector3(0.0, -0.035, 0.0));   
    temp.setValue(1,0,0,
                0,1,0,
                0,0,1);
    finger_2_1_tf.setBasis(temp);



    tf::Transform finger_1_1_obj_tf, finger_2_1_obj_tf;
    finger_1_1_obj_tf = palm_obj_tf * finger_1_1_tf;    
    finger_2_1_obj_tf = palm_obj_tf * finger_2_1_tf;
  
    tf::Vector3 touch_1_1, touch_1_2, touch_1_3, touch_2_1, touch_2_2, touch_2_3;
    touch_1_1 = finger_1_1_obj_tf * tf::Vector3(0.02,0.0,0.0);
    collision_points.push_back(touch_1_1);
    touch_1_2 = finger_1_1_obj_tf * tf::Vector3(0.04,0.0,0.0);
    collision_points.push_back(touch_1_2);
    touch_1_3 = finger_1_1_obj_tf * tf::Vector3(0.06,0.0,0.0);
    collision_points.push_back(touch_1_3);
    touch_2_1 = finger_2_1_obj_tf * tf::Vector3(0.02,0.0,0.0);
    collision_points.push_back(touch_2_1);
    touch_2_2 = finger_2_1_obj_tf * tf::Vector3(0.04,0.0,0.0);
    collision_points.push_back(touch_2_2);
    touch_2_3 = finger_2_1_obj_tf * tf::Vector3(0.06,0.0,0.0);
    collision_points.push_back(touch_2_3);

    

  }  // end for the m2 case
}

/*********************************************
check collision between object_1 and object_2, object_1 and object_2 should be the general name for the object, such 
as "cube" and "collision_gripper".
Return true if there is collision
       false if no collision
***********************************************/
bool GraspGenerator::checkCollision(std::string object_1, std::string object_2,fcl::Vec3f T1, fcl::Vec3f T2, fcl::Matrix3f R1, fcl::Matrix3f R2){
  std::vector<fcl::Vec3f> vertices1;
  std::vector<fcl::Triangle> triangles1;
  std::vector<fcl::Vec3f> vertices2;
  std::vector<fcl::Triangle> triangles2;
  #ifdef DEBUG_
  ROS_INFO_STREAM("checkCollision:"<<object_1<<" and "<<object_2);
  #endif
  if(object_1 == "cube"){
    vertices1 = vertices_cube_;
    triangles1 = triangles_cube_;
  }else if(object_1 == "collision_gripper"){
    vertices1 = vertices_collision_gripper_;
    triangles1 = triangles_collision_gripper_;
  }else if(object_1 == "floor"){
    vertices1 = vertices_floor_;
    triangles1 = triangles_floor_;
  }else if(object_1 == "base_frame_convex"){
    vertices1 = vertices_base_;
    triangles1 = triangles_base_;
  }else if(object_1 == "opti_cube"){
    vertices1 = vertices_opti_cube_;
    triangles1 = triangles_opti_cube_;
  }else if(object_1 == "standard_gripper"){
    vertices1 = vertices_standard_gripper_;
    triangles1 = triangles_standard_gripper_;
  }else if(object_1 == "arm0"){
    vertices1 = vertices_arm0_;
    triangles1 = triangles_arm0_;
  }else if(object_1 == "arm1"){
    vertices1 = vertices_arm1_;
    triangles1 = triangles_arm1_;
  }else if(object_1 == "arm2"){
    vertices1 = vertices_arm2_;
    triangles1 = triangles_arm2_;
  }else if(object_1 == "arm3"){
    vertices1 = vertices_arm3_;
    triangles1 = triangles_arm3_;
  }else if(object_1 == "arm4"){
    vertices1 = vertices_arm4_;
    triangles1 = triangles_arm4_;
  }else if(object_1 == "arm5"){
    vertices1 = vertices_arm5_new_;
    triangles1 = triangles_arm5_new_;
  }

  if(object_2 == "cube"){
    vertices2 = vertices_cube_;
    triangles2 = triangles_cube_;
  }else if(object_2 == "collision_gripper"){
    vertices2 = vertices_collision_gripper_;
    triangles2 = triangles_collision_gripper_;
  }else if(object_2 == "floor"){
    vertices2 = vertices_floor_;
    triangles2 = triangles_floor_;
  }else if(object_2 == "base_frame_convex"){
    vertices2 = vertices_base_;
    triangles2 = triangles_base_;
  }else if(object_2 == "opti_cube"){
    vertices2 = vertices_opti_cube_;
    triangles2 = triangles_opti_cube_;
  }else if(object_2 == "standard_gripper"){
    vertices2 = vertices_standard_gripper_;
    triangles2 = triangles_standard_gripper_;
  }else if(object_2 == "arm0"){
    vertices2 = vertices_arm0_;
    triangles2 = triangles_arm0_;
  }else if(object_2 == "arm1"){
    vertices2 = vertices_arm1_;
    triangles2 = triangles_arm1_;
  }else if(object_2 == "arm2"){
    vertices2 = vertices_arm2_;
    triangles2 = triangles_arm2_;
  }else if(object_2 == "arm3"){
    vertices2 = vertices_arm3_;
    triangles2 = triangles_arm3_;
  }else if(object_2 == "arm4"){
    vertices2 = vertices_arm4_;
    triangles2 = triangles_arm4_;
  }else if(object_2 == "arm5"){
    vertices2 = vertices_arm5_new_;
    triangles2 = triangles_arm5_new_;
  }
  typedef fcl::BVHModel<fcl::OBBRSS> Model;
  Model* m1 = new Model();
  Model* m2 = new Model();

  m1->beginModel();
  m1->addSubModel(vertices1, triangles1);
  m1->endModel();

  m2->beginModel();
  m2->addSubModel(vertices2, triangles2);
  m2->endModel();

 
  fcl::Transform3f pose1;
  pose1.setIdentity();
  pose1.setRotation(R1);
  pose1.setTranslation(T1);
    
  fcl::Transform3f pose2;    
  pose2.setIdentity();
  pose2.setRotation(R2);
  pose2.setTranslation(T2);
  
  fcl::CollisionObject* obj1 = new fcl::CollisionObject(boost::shared_ptr<fcl::BVHModel<fcl::OBBRSS>>(m1), pose1);
  fcl::CollisionObject* obj2 = new fcl::CollisionObject(boost::shared_ptr<fcl::BVHModel<fcl::OBBRSS>>(m2), pose2);
  fcl::CollisionRequest request;
// result will be returned via the collision result structure
  fcl::CollisionResult result;
// perform collision test
  int num_contacts = fcl::collide(m1, pose1, m2, pose2, request, result);
  if(num_contacts > 0){
    #ifdef DEBUG_
    ROS_INFO_STREAM("collision happen!");
    #endif
    return true;
  }else{
    #ifdef DEBUG_
    ROS_INFO_STREAM("no collision");
    #endif
    return false;
  }

}     


void GraspGenerator::geoPoseToFclPose(geometry_msgs::Pose p, fcl::Vec3f &v, fcl::Matrix3f &m){ 

  fcl::Vec3f  T1(p.position.x, p.position.y, p.position.z);
  tf::Quaternion tf_target_q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
  tf::Matrix3x3 tf_target_m(tf_target_q);
  fcl::Matrix3f fcl_target_m;
  fcl_target_m(0,0) = tf_target_m.getRow(0)[0];
  fcl_target_m(0,1) = tf_target_m.getRow(0)[1];
  fcl_target_m(0,2) = tf_target_m.getRow(0)[2];
  fcl_target_m(1,0) = tf_target_m.getRow(1)[0];
  fcl_target_m(1,1) = tf_target_m.getRow(1)[1];
  fcl_target_m(1,2) = tf_target_m.getRow(1)[2];
  fcl_target_m(2,0) = tf_target_m.getRow(2)[0];
  fcl_target_m(2,1) = tf_target_m.getRow(2)[1];
  fcl_target_m(2,2) = tf_target_m.getRow(2)[2];
  
  v = T1;
  m = fcl_target_m;

}
/*********************************************
obtain the quaternion representation of the transform between contact point and the target object
grasp_point and grasp_dir are returned candidates from the dora_grasp planning service.  
  
***********************************************/


tf::Matrix3x3 GraspGenerator::get_cp_obj_quaternion(geometry_msgs::Point grasp_point, geometry_msgs::Point grasp_dir, double rotate_around_x)
{
  // if(grasp_point.x * grasp_dir.x + grasp_point.y * grasp_dir.y + grasp_point.z * grasp_dir.z   > 0)
  // {
  grasp_dir.x = - grasp_dir.x;
  grasp_dir.y = - grasp_dir.y;
  grasp_dir.z = - grasp_dir.z;
  // }
  double a, b, c, temp, cos_ro, sin_ro;//cos_ro, sin_ro are the cos and sin of the rotation angle around x-axis
  a = grasp_dir.x;
  b = grasp_dir.y;
  c = grasp_dir.z;
  temp = sqrt(pow(c,2) + pow(a,2));

  // ROS_INFO_STREAM(a);
  // ROS_INFO_STREAM(b);
  // ROS_INFO_STREAM(c);

  geometry_msgs::Point y_axis, z_axis, x_axis, ro_y_axis, ro_z_axis, ro_x_axis, new_x_axis, new_y_axis, new_z_axis, new_ro_x_axis, new_ro_y_axis, new_ro_z_axis;
  cos_ro = cos(rotate_around_x);
  sin_ro = sin(rotate_around_x);

  if(a != 0){ 
    if(b==0){ 
  
    z_axis.x = -b*a / temp;
    z_axis.y = temp;
    z_axis.z = -b*c / temp;

    x_axis.x = a;
    x_axis.y = b;
    x_axis.z = c;

    y_axis.x =z_axis.y*x_axis.z - x_axis.y*z_axis.z;
    y_axis.y = x_axis.x*z_axis.z - z_axis.x*x_axis.z;
    y_axis.z = z_axis.x*x_axis.y - x_axis.x*z_axis.y;
    }else if(c == 0){
      temp = sqrt(pow(b,2) + pow(a,2));
      z_axis.x = 0;
      z_axis.y = 0;
      z_axis.z = temp;

      x_axis.x = a;
      x_axis.y = b;
      x_axis.z = c;
      y_axis.x =z_axis.y*x_axis.z - x_axis.y*z_axis.z;
    y_axis.y = x_axis.x*z_axis.z - z_axis.x*x_axis.z;
    y_axis.z = z_axis.x*x_axis.y - x_axis.x*z_axis.y;
    }
    // y_axis.x = -sqrt(pow(a,2))/temp * (c/a);
    // y_axis.y = 0;
    // y_axis.z = sqrt(pow(a,2))/temp;
  }else{
    y_axis.y = 0;
    y_axis.z = 0;
    if(b> 0 && c > 0){
      y_axis.x = 1;
    }else if(b > 0 && c < 0){
      y_axis.x = -1;
    }else if(b < 0 && c > 0){
      y_axis.x = 1;
    }else if(b < 0 && c < 0){
      y_axis.x = -1;
    }
    if(b == 0){
      if(c > 0){
        y_axis.x = -1;
      }else{
        y_axis.x = 1;
      }

    }
    if(c != 0){ 
      

      x_axis.x = a;
      x_axis.y = b;
      x_axis.z = c;
      z_axis.x = x_axis.y*y_axis.z - y_axis.y*x_axis.z;
      z_axis.y = y_axis.x*x_axis.z - x_axis.x*y_axis.z;
      z_axis.z = x_axis.x*y_axis.y - y_axis.x*x_axis.y;
      // z_axis.x = -b*a / temp;
      // z_axis.y = -temp;
      // z_axis.z = -b*c / temp;
    }else{
      if(b < 0){
        y_axis.x = 1;
        z_axis.x = 0;
        z_axis.y = 0;
        z_axis.z = 1;
        x_axis.x = 0;
        x_axis.y = -1;
        x_axis.z = 0;
      }else{
        y_axis.x = -1;
        z_axis.x = 0;
        z_axis.y = 0;
        z_axis.z = 1;
        x_axis.x = 0;
        x_axis.y = 1;
        x_axis.z = 0;
      }
    }

  }


  

  new_z_axis.x = -x_axis.x;
  new_z_axis.y = -x_axis.y;
  new_z_axis.z = -x_axis.z;

  new_y_axis.x = -z_axis.x;
  new_y_axis.y = -z_axis.y;
  new_y_axis.z = -z_axis.z;

  new_x_axis.x = y_axis.x;
  new_x_axis.y = y_axis.y;
  new_x_axis.z = y_axis.z; 

  // rotation matrix after rotating around x-axis, the rotation is of the palm
  new_ro_z_axis.x = new_z_axis.x;
  new_ro_z_axis.y = new_z_axis.y;
  new_ro_z_axis.z = new_z_axis.z;

  new_ro_x_axis.x = new_x_axis.x*cos_ro + new_y_axis.x*sin_ro;
  new_ro_x_axis.y = new_x_axis.y*cos_ro + new_y_axis.y*sin_ro;
  new_ro_x_axis.z = new_x_axis.z*cos_ro + new_y_axis.z*sin_ro;

  new_ro_y_axis.x = -new_x_axis.x*sin_ro + new_y_axis.x*cos_ro;
  new_ro_y_axis.y = -new_x_axis.y*sin_ro + new_y_axis.y*cos_ro;
  new_ro_y_axis.z = -new_x_axis.z*sin_ro + new_y_axis.z*cos_ro;

  // ro_x_axis.x = x_axis.x;
  // ro_x_axis.y = x_axis.y;
  // ro_x_axis.z = x_axis.z;

  // ro_y_axis.x = y_axis.x*cos_ro+z_axis.x*sin_ro;
  // ro_y_axis.y = y_axis.y*cos_ro+z_axis.y*sin_ro;
  // ro_y_axis.z = y_axis.z*cos_ro+z_axis.z*sin_ro;

  // ro_z_axis.x = -y_axis.x*sin_ro + z_axis.x*cos_ro;
  // ro_z_axis.y = -y_axis.y*sin_ro + z_axis.y*cos_ro;
  // ro_z_axis.z = -y_axis.z*sin_ro + z_axis.z*cos_ro;



  // derive the quaternion from the rotation matrix
  // double q_x, q_y, q_z, q_w;
  // double trace = a + ro_y_axis.y + ro_z_axis.z;
  // if(trace > 0)
  // {
  //   double s = 0.5 / sqrt(trace + 1);
  //   q_w = 0.25 / s;
  //   q_x = (ro_y_axis.z - ro_z_axis.y) * s;
  //   q_y = (ro_z_axis.x - ro_x_axis.z) * s;
  //   q_z = (ro_x_axis.y - ro_y_axis.x) * s; 
  // }
  // else
  // {
  //   if(ro_x_axis.x > ro_y_axis.y && ro_x_axis.x > ro_z_axis.z)
  //   {
  //     double s = 2.0 * sqrt(1.0 + ro_x_axis.x - ro_y_axis.y - ro_z_axis.z);
  //     q_w = (ro_y_axis.z - ro_z_axis.y) / s;
  //     q_x = 0.25 * s;
  //     q_y = (ro_y_axis.x + ro_x_axis.y) / s;
  //     q_z = (ro_z_axis.x + ro_x_axis.z) / s;

  //   }
  //   else if(ro_y_axis.y > ro_z_axis.z)
  //   {
  //     double s = 2.0 * sqrt(1.0 + ro_y_axis.y - ro_x_axis.x - ro_z_axis.z);
  //     q_w = (ro_z_axis.x - ro_x_axis.z) / s;
  //     q_x = (ro_y_axis.x + ro_x_axis.y) / s;
  //     q_y = 0.25 * s;
  //     q_z = (ro_z_axis.y + ro_y_axis.z) / s;

  //   }
  //   else
  //   {
  //     double s = 2.0 * sqrt(1.0 + ro_z_axis.z - ro_x_axis.x - ro_y_axis.y);
  //     q_w = (ro_x_axis.y - ro_y_axis.x) / s;
  //     q_x = (ro_z_axis.x + ro_x_axis.z) / s;
  //     q_y = (ro_z_axis.y + ro_y_axis.z) / s;
  //     q_z = 0.25 * s;
  //   }  
  // }
  tf::Quaternion final_q;
  tf::Matrix3x3 final_matrix(new_ro_x_axis.x,new_ro_y_axis.x, new_ro_z_axis.x,new_ro_x_axis.y, new_ro_y_axis.y,new_ro_z_axis.y, new_ro_x_axis.z, new_ro_y_axis.z, new_ro_z_axis.z );
  final_matrix.getRotation(final_q);
  return final_matrix;
  // derive the quaternion from the rotation matrix
  // double q_x, q_y, q_z, q_w;
  // double trace = new_ro_x_axis.x + new_ro_y_axis.y + new_ro_z_axis.z;
  // if(trace > 0)
  // {
  //   double s = 0.5 / sqrt(trace + 1);
  //   q_w = 0.25 / s;
  //   q_x = (new_ro_y_axis.z - new_ro_z_axis.y) * s;
  //   q_y = (new_ro_z_axis.x - new_ro_x_axis.z) * s;
  //   q_z = (new_ro_x_axis.y - new_ro_y_axis.x) * s; 
  // }
  // else
  // {
  //   if(new_ro_x_axis.x > new_ro_y_axis.y && new_ro_x_axis.x > new_ro_z_axis.z)
  //   {
  //     double s = 2.0 * sqrt(1.0 + new_ro_x_axis.x - new_ro_y_axis.y - new_ro_z_axis.z);
  //     q_w = (new_ro_y_axis.z - new_ro_z_axis.y) / s;
  //     q_x = 0.25 * s;
  //     q_y = (new_ro_y_axis.x + new_ro_x_axis.y) / s;
  //     q_z = (new_ro_z_axis.x + new_ro_x_axis.z) / s;

  //   }
  //   else if(new_ro_y_axis.y > new_ro_z_axis.z)
  //   {
  //     double s = 2.0 * sqrt(1.0 + new_ro_y_axis.y - new_ro_x_axis.x - new_ro_z_axis.z);
  //     q_w = (new_ro_z_axis.x - new_ro_x_axis.z) / s;
  //     q_x = (new_ro_y_axis.x + new_ro_x_axis.y) / s;
  //     q_y = 0.25 * s;
  //     q_z = (new_ro_z_axis.y + new_ro_y_axis.z) / s;

  //   }
  //   else
  //   {
  //     double s = 2.0 * sqrt(1.0 + new_ro_z_axis.z - new_ro_x_axis.x - new_ro_y_axis.y);
  //     q_w = (new_ro_x_axis.y - new_ro_y_axis.x) / s;
  //     q_x = (new_ro_z_axis.x + new_ro_x_axis.z) / s;
  //     q_y = (new_ro_z_axis.y + new_ro_y_axis.z) / s;
  //     q_z = 0.25 * s;
  //   }  
  // }


  // return tf::Quaternion(q_x, q_y, q_z, q_w);
}
/*
* 
*  version for fcl 0.6.0

template <typename S>
void GraspGenerator::loadOBJFile(const char* filename, std::vector<fcl::Vector3<S>>& points, std::vector<fcl::Triangle>& triangles)
{

  FILE* file = fopen(filename, "rb");
  if(!file)
  {
    std::cerr << "file not exist" << std::endl;
    return;
  }

  bool has_normal = false;
  bool has_texture = false;
  char line_buffer[2000];
  while(fgets(line_buffer, 2000, file))
  {
    char* first_token = strtok(line_buffer, "\r\n\t ");
    if(!first_token || first_token[0] == '#' || first_token[0] == 0)
      continue;

    switch(first_token[0])
    {
    case 'v':
      {
        if(first_token[1] == 'n')
        {
          strtok(nullptr, "\t ");
          strtok(nullptr, "\t ");
          strtok(nullptr, "\t ");
          has_normal = true;
        }
        else if(first_token[1] == 't')
        {
          strtok(nullptr, "\t ");
          strtok(nullptr, "\t ");
          has_texture = true;
        }
        else
        {
          S x = (S)atof(strtok(nullptr, "\t "))/1000;
          S y = (S)atof(strtok(nullptr, "\t "))/1000;
          S z = (S)atof(strtok(nullptr, "\t "))/1000;
          points.emplace_back(x, y, z);
        }
      }
      break;
    case 'f':
      {
        fcl::Triangle tri;
        char* data[30];
        int n = 0;
        while((data[n] = strtok(nullptr, "\t \r\n")) != nullptr)
        {
          if(strlen(data[n]))
            n++;
        }

        for(int t = 0; t < (n - 2); ++t)
        {
          if((!has_texture) && (!has_normal))
          {
            tri[0] = atoi(data[0]) - 1;
            tri[1] = atoi(data[1]) - 1;
            tri[2] = atoi(data[2]) - 1;
          }
          else
          {
            const char *v1;
            for(int i = 0; i < 3; i++)
            {
              // vertex ID
              if(i == 0)
                v1 = data[0];
              else
                v1 = data[t + i];

              tri[i] = atoi(v1) - 1;
            }
          }
          triangles.push_back(tri);
        }
      }
    }
  }
}
*/


void GraspGenerator::loadOBJFile(const char* filename, std::vector<fcl::Vec3f>& points, std::vector<fcl::Triangle>& triangles)
{

  FILE* file = fopen(filename, "rb");
  if(!file)
  {
    std::cerr << "file not exist" << std::endl;
    return;
  }

  bool has_normal = false;
  bool has_texture = false;
  char line_buffer[2000];
  while(fgets(line_buffer, 2000, file))
  {
    char* first_token = strtok(line_buffer, "\r\n\t ");
    if(!first_token || first_token[0] == '#' || first_token[0] == 0)
      continue;

    switch(first_token[0])
    {
    case 'v':
      {
        if(first_token[1] == 'n')
        {
          strtok(nullptr, "\t ");
          strtok(nullptr, "\t ");
          strtok(nullptr, "\t ");
          has_normal = true;
        }
        else if(first_token[1] == 't')
        {
          strtok(nullptr, "\t ");
          strtok(nullptr, "\t ");
          has_texture = true;
        }
        else
        {
          double x = atof(strtok(nullptr, "\t "))/1000;
          double y = atof(strtok(nullptr, "\t "))/1000;
          double z = atof(strtok(nullptr, "\t "))/1000;
          points.emplace_back(x, y, z);
        }
      }
      break;
    case 'f':
      {
        fcl::Triangle tri;
        char* data[30];
        int n = 0;
        while((data[n] = strtok(nullptr, "\t \r\n")) != nullptr)
        {
          if(strlen(data[n]))
            n++;
        }

        for(int t = 0; t < (n - 2); ++t)
        {
          if((!has_texture) && (!has_normal))
          {
            tri[0] = atoi(data[0]) - 1;
            tri[1] = atoi(data[1]) - 1;
            tri[2] = atoi(data[2]) - 1;
          }
          else
          {
            const char *v1;
            for(int i = 0; i < 3; i++)
            {
              // vertex ID
              if(i == 0)
                v1 = data[0];
              else
                v1 = data[t + i];

              tri[i] = atoi(v1) - 1;
            }
          }
          triangles.push_back(tri);
        }
      }
    }
  }
}
