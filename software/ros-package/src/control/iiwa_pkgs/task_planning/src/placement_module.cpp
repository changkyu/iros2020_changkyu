#include <placement_module.hpp>
#define USE_DOVE
//#define USE_TOOTH
// void  placement_module::setup(std::string target_object_name)
// {

// 	ROS_ERROR_STREAM("Setting up the placement module...");
// /*
// //1st version, handcrafted placement position

// 	std::vector<geometry_msgs::Pose> grid_pose_vector_bin_1, grid_pose_vector_bin_2;
// 	geometry_msgs::Pose bin_1_center, bin_2_center;


// 	bin_2_center.position.x = 0.423;
// 	bin_2_center.position.y = 0.342;
// 	bin_2_center.position.z = 0.46;
// 	bin_2_center.orientation.x = -0.334696;
// 	bin_2_center.orientation.y = 0.94232;
// 	bin_2_center.orientation.z = -0.00172;
// 	bin_2_center.orientation.w = 0.00083;
// 	bin_1_center.position.x = 0.4088;
// 	bin_1_center.position.y = -0.329;
// 	bin_1_center.position.z = 0.46;
// 	bin_1_center.orientation.x = 0.34554;
// 	bin_1_center.orientation.y = 0.93840;
// 	bin_1_center.orientation.z = -0.001849;
// 	bin_1_center.orientation.w = -0.000465;
// 	std::pair<double, double> bin_1_x_axis, bin_1_y_axis, bin_2_x_axis, bin_2_y_axis;
// 	bin_2_x_axis.first = 0.5;
// 	bin_2_x_axis.second = -0.866025404;
// 	bin_2_y_axis.first = 0.866025404;
// 	bin_2_y_axis.second = 0.5;
// 	bin_1_x_axis.first = -0.5;
// 	bin_1_x_axis.second = -0.866025404;
// 	bin_1_y_axis.first = 0.866025404;
// 	bin_1_y_axis.second = -0.5;
// 	double bin_2_x_length = 0.55;
// 	double bin_2_y_length = 0.37;
// 	double bin_1_x_length = 0.2921;
// 	double bin_1_y_length = 0.21082;
	
// 	std::map<int, std::pair<double, double>> bin_1_id_relative_pose_map, bin_2_id_relative_pose_map;
// 	for(int j = 0; j < 4; j++){
// 		for(int i = 0; i < 3; i++){
// 			int id = 3*j + i;
// 			std::pair<double, double> relative_pose;
// 			relative_pose.first = bin_2_x_length / 5.0 * (j+1)  - bin_2_x_length * 0.5; 
// 			relative_pose.second = (bin_2_y_length - 0.05)*0.5 - (bin_2_y_length-0.05) / 4.0 *(i+1); 
// 			//bin_1_id_relative_pose_map[id] = relative_pose;
// 			bin_2_id_relative_pose_map[id] = relative_pose;
// 		}
// 	}
// 	for(int i = 0; i < 3; i++){
// 		for(int j = 0; j < 3; j++){
// 			int id = 3*i + j;
// 			std::pair<double, double> relative_pose;
// 			relative_pose.first = bin_1_x_length / 6.0 * (2*i+1)  - bin_1_x_length * 0.5; 
// 			relative_pose.second = (bin_1_y_length) / 6.0 *(2*j+1) - (bin_1_y_length)*0.5; 
// 			bin_1_id_relative_pose_map[id] = relative_pose;
// 			//bin_2_id_relative_pose_map[id] = relative_pose;
// 		}
// 	}


// 	// for bin_1 pose
// 	for(int j = 0; j < 4; j ++){
// 		for(int i = 0; i < 3; i ++){
// 			int id = 3*j + i;
// 			geometry_msgs::Pose temp;
// 			temp = bin_2_center;
// 			temp.position.x = bin_2_center.position.x + bin_2_x_axis.first*bin_2_id_relative_pose_map[id].first + bin_2_y_axis.first * bin_2_id_relative_pose_map[id].second;
// 			temp.position.y = bin_2_center.position.y + bin_2_x_axis.second*bin_2_id_relative_pose_map[id].first + bin_2_y_axis.second * bin_2_id_relative_pose_map[id].second; 
// 			//bin_2_id_pose_map[id] = temp;
// 			bin_id_pose_map["bin_2"][id] = temp;
// 			// ROS_WARN_STREAM("id_"<<id<<": ["<<bin_id_pose_map["bin_2"][id].position.x<<","<<bin_id_pose_map["bin_2"][id].position.y<<","<<bin_id_pose_map["bin_2"][id].position.z<<"]");
// 		}

// 	}
// 	for(int i = 0; i < 3; i ++){
// 		for(int j = 0; j < 3; j ++){
// 			int id = 3*i + j;
// 			geometry_msgs::Pose temp;
// 			temp = bin_1_center;
// 			temp.position.x = bin_1_center.position.x + bin_1_x_axis.first*bin_1_id_relative_pose_map[id].first + bin_1_y_axis.first * bin_1_id_relative_pose_map[id].second;
// 			temp.position.y = bin_1_center.position.y + bin_1_x_axis.second*bin_1_id_relative_pose_map[id].first + bin_1_y_axis.second * bin_1_id_relative_pose_map[id].second; 
// 			//bin_1_id_pose_map[id] = temp;
// 			bin_id_pose_map["bin_1"][id] = temp;
// 			// ROS_WARN_STREAM("id_"<<id<<": ["<<bin_id_pose_map["bin_1"][id].position.x<<","<<bin_id_pose_map["bin_1"][id].position.y<<","<<bin_id_pose_map["bin_1"][id].position.z<<"]");
// 		}

// 	}
// */
// 	char obj_topic[50];
//     std::string obj_name;
//     double obj_length = 0;
//     double obj_width = 0;
//     double z_length = 0;
//     int num_objects = 0;
  
    
//     if(nh.hasParam("/iiwa/objects/num_objects"))
//       nh.getParam("/iiwa/objects/num_objects", num_objects);
//     else
//     {
//       ROS_WARN_STREAM("Could not read the object config file...");
//       exit(1);
//     }
//     for(int ii = 0; ii < num_objects; ii++)
//     { 
//       sprintf(obj_topic, "/objects/object_%d", ii+1);
//       nh.getParam((std::string(obj_topic) + "/name").c_str(), obj_name);
//       if(obj_name == target_object_name){
//         nh.getParam((std::string(obj_topic) + "/x_dimension").c_str(), obj_length);
//         nh.getParam((std::string(obj_topic) + "/y_dimension").c_str(), obj_width); 
//         nh.getParam((std::string(obj_topic) + "/z_dimension").c_str(), z_length); 
//         break;
//       }
//     }

// 	std::vector<geometry_msgs::Pose> grid_pose_vector_bin_1, grid_pose_vector_bin_2;
// 	geometry_msgs::Pose bin_1_center, bin_2_center;

// 	bin_2_center.position.x = 0.423;
// 	bin_2_center.position.y = 0.342;
// 	bin_2_center.position.z = 0.46;
// 	bin_2_center.orientation.x = -0.334696;
// 	bin_2_center.orientation.y = 0.94232;
// 	bin_2_center.orientation.z = -0.00172;
// 	bin_2_center.orientation.w = 0.00083;
// 	// bin_1_center.position.x = 0.4088;
// 	// bin_1_center.position.y = -0.329;
// 	//bin_1_center.position.x = 0.4088  - 0.01;
// 	//bin_1_center.position.y = -0.329 + 0.005;
// 	bin_1_center.position.x = 0.4088  - 0.01 - 0.01;
//  	bin_1_center.position.y = -0.329 + 0.005 + 0.0172;
// 	// bin_1_center.position.x = 0.4088  - 0.01 + 0.01;
// 	// bin_1_center.position.y = -0.329 + 0.005 + 0.0172;
// 	// bin_1_center.position.x = 0.4088  - 0.01 - 0.01 - 0.0086;
// 	// bin_1_center.position.y = -0.329 + 0.005 - 0.0172 + 0.005;
// 	bin_1_center.position.z = 0.46;
// 	bin_1_center.orientation.x = 0.34554;
// 	bin_1_center.orientation.y = 0.93840;
// 	bin_1_center.orientation.z = -0.001849;
// 	bin_1_center.orientation.w = -0.000465;
// 	std::pair<double, double> bin_1_x_axis, bin_1_y_axis, bin_2_x_axis, bin_2_y_axis;
// 	bin_2_x_axis.first = 0.5;
// 	bin_2_x_axis.second = -0.866025404;
// 	bin_2_y_axis.first = 0.866025404;
// 	bin_2_y_axis.second = 0.5;
// 	bin_1_x_axis.first = -0.5;
// 	bin_1_x_axis.second = -0.866025404;
// 	bin_1_y_axis.first = 0.866025404;
// 	bin_1_y_axis.second = -0.5;
// 	double bin_2_x_length = 0.55;
// 	double bin_2_y_length = 0.37;
// 	double bin_1_x_length = 0.2921;
// 	double bin_1_y_length = 0.21082;
	
// 	std::map<int, std::pair<double, double>> bin_1_id_relative_pose_map, bin_2_id_relative_pose_map;

// 	int bin_1_x_num, bin_1_y_num, bin_1_total_num;
// 	bool is_align_with_bin_1 = true;
// 	bool is_align_with_bin_2 = true;
// 	bin_1_x_num = bin_1_x_length / obj_length;
// 	bin_1_y_num = bin_1_y_length / obj_width;
// 	bin_1_total_num = bin_1_x_num * bin_1_y_num;
// 	ROS_WARN_STREAM("obj_length:"<<obj_length);
// 	ROS_WARN_STREAM("obj_width:"<<obj_width);
// 	ROS_WARN_STREAM("bin_1_total_num:"<<bin_1_total_num);
// 	bin_1_x_num = bin_1_x_length / obj_width;
// 	bin_1_y_num = bin_1_y_length / obj_length;
// 	if(bin_1_x_num * bin_1_y_num > bin_1_total_num){
// 		bin_1_total_num = bin_1_x_num * bin_1_y_num;
// 		is_align_with_bin_1 = false;
// 	}else{
// 		bin_1_x_num = bin_1_x_length / obj_length;
// 		bin_1_y_num = bin_1_y_length / obj_width;
// 	}

// 	for(int j = 0; j < 4; j++){
// 		for(int i = 0; i < 3; i++){
// 			int id = 3*j + i;
// 			std::pair<double, double> relative_pose;
// 			relative_pose.first = bin_2_x_length / 5.0 * (j+1) - bin_2_x_length * 0.5; 
// 			relative_pose.second = (bin_2_y_length - 0.05)*0.5 - (bin_2_y_length-0.05) / 4.0 *(i+1); 
// 			bin_2_id_relative_pose_map[id] = relative_pose;
// 		}
// 	}

// 	for(int i = 0; i < bin_1_x_num; i++){
// 		for(int j = 0; j < bin_1_y_num; j++){
// 			int id = bin_1_y_num*i + j;
// 			std::pair<double, double> relative_pose;
// 			relative_pose.first = bin_1_x_length / (2*bin_1_x_num) * (2*i+1)  - bin_1_x_length * 0.5; 
// 			relative_pose.second = (bin_1_y_length) / (2*bin_1_y_num) *(2*j+1) - (bin_1_y_length)*0.5; 
// 			bin_1_id_relative_pose_map[id] = relative_pose;
// 		}
// 	}

// 	for(int j = 0; j < 4; j ++){
// 		for(int i = 0; i < 3; i ++){
// 			int id = 3*j + i;
// 			geometry_msgs::Pose temp;
// 			temp = bin_2_center;
// 			temp.position.x = bin_2_center.position.x + bin_2_x_axis.first*bin_2_id_relative_pose_map[id].first + bin_2_y_axis.first * bin_2_id_relative_pose_map[id].second;
// 			temp.position.y = bin_2_center.position.y + bin_2_x_axis.second*bin_2_id_relative_pose_map[id].first + bin_2_y_axis.second * bin_2_id_relative_pose_map[id].second; 
// 			//bin_2_id_pose_map[id] = temp;
			
// 			bin_id_pose_map["bin_2"][id] = temp;

// 			// ROS_WARN_STREAM("id_"<<id<<": ["<<bin_id_pose_map["bin_2"][id].position.x<<","<<bin_id_pose_map["bin_2"][id].position.y<<","<<bin_id_pose_map["bin_2"][id].position.z<<"]");
// 		}

// 	}
// 	for(int i = 0; i < bin_1_x_num; i ++){
// 		for(int j = 0; j < bin_1_y_num; j ++){
// 			int id = bin_1_y_num*i + j;
// 			geometry_msgs::Pose temp;
// 			temp = bin_1_center;
// 			temp.position.x = bin_1_center.position.x + bin_1_x_axis.first*bin_1_id_relative_pose_map[id].first + bin_1_y_axis.first * bin_1_id_relative_pose_map[id].second;
// 			temp.position.y = bin_1_center.position.y + bin_1_x_axis.second*bin_1_id_relative_pose_map[id].first + bin_1_y_axis.second * bin_1_id_relative_pose_map[id].second; 
// 			//bin_1_id_pose_map[id] = temp;
// 			if(is_align_with_bin_1){
// 				temp.orientation.x = 0;
// 				temp.orientation.y = 0;
// 				temp.orientation.z = 0.5;
// 				temp.orientation.w = 0.8660254;
// 			}else{
// 				temp.orientation.x = 0;
// 				temp.orientation.y = 0;
// 				temp.orientation.z = 0.9659258;
// 				temp.orientation.w = 0.258819;
// 			}
// 			bin_id_pose_map["bin_1"][id] = temp;

// 			ROS_WARN_STREAM("id_"<<id<<": ["<<bin_id_pose_map["bin_1"][id].position.x<<","<<bin_id_pose_map["bin_1"][id].position.y<<","<<bin_id_pose_map["bin_1"][id].position.z<<"]");
// 		}

// 	}

// 	ROS_ERROR_STREAM("Reading the object models...");
// 	read_object_models();


// 	ROS_ERROR_STREAM("Setting up the placement module...");

// }

int  placement_module::setup(std::string target_object_name)
{

	ROS_ERROR_STREAM("Setting up the placement module...");
/*
//1st version, handcrafted placement position

	std::vector<geometry_msgs::Pose> grid_pose_vector_bin_1, grid_pose_vector_bin_2;
	geometry_msgs::Pose bin_1_center, bin_2_center;


	bin_2_center.position.x = 0.423;
	bin_2_center.position.y = 0.342;
	bin_2_center.position.z = 0.46;
	bin_2_center.orientation.x = -0.334696;
	bin_2_center.orientation.y = 0.94232;
	bin_2_center.orientation.z = -0.00172;
	bin_2_center.orientation.w = 0.00083;
	bin_1_center.position.x = 0.4088;
	bin_1_center.position.y = -0.329;
	bin_1_center.position.z = 0.46;
	bin_1_center.orientation.x = 0.34554;
	bin_1_center.orientation.y = 0.93840;
	bin_1_center.orientation.z = -0.001849;
	bin_1_center.orientation.w = -0.000465;
	std::pair<double, double> bin_1_x_axis, bin_1_y_axis, bin_2_x_axis, bin_2_y_axis;
	bin_2_x_axis.first = 0.5;
	bin_2_x_axis.second = -0.866025404;
	bin_2_y_axis.first = 0.866025404;
	bin_2_y_axis.second = 0.5;
	bin_1_x_axis.first = -0.5;
	bin_1_x_axis.second = -0.866025404;
	bin_1_y_axis.first = 0.866025404;
	bin_1_y_axis.second = -0.5;
	double bin_2_x_length = 0.55;
	double bin_2_y_length = 0.37;
	double bin_1_x_length = 0.2921;
	double bin_1_y_length = 0.21082;
	
	std::map<int, std::pair<double, double>> bin_1_id_relative_pose_map, bin_2_id_relative_pose_map;
	for(int j = 0; j < 4; j++){
		for(int i = 0; i < 3; i++){
			int id = 3*j + i;
			std::pair<double, double> relative_pose;
			relative_pose.first = bin_2_x_length / 5.0 * (j+1)  - bin_2_x_length * 0.5; 
			relative_pose.second = (bin_2_y_length - 0.05)*0.5 - (bin_2_y_length-0.05) / 4.0 *(i+1); 
			//bin_1_id_relative_pose_map[id] = relative_pose;
			bin_2_id_relative_pose_map[id] = relative_pose;
		}
	}
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			int id = 3*i + j;
			std::pair<double, double> relative_pose;
			relative_pose.first = bin_1_x_length / 6.0 * (2*i+1)  - bin_1_x_length * 0.5; 
			relative_pose.second = (bin_1_y_length) / 6.0 *(2*j+1) - (bin_1_y_length)*0.5; 
			bin_1_id_relative_pose_map[id] = relative_pose;
			//bin_2_id_relative_pose_map[id] = relative_pose;
		}
	}


	// for bin_1 pose
	for(int j = 0; j < 4; j ++){
		for(int i = 0; i < 3; i ++){
			int id = 3*j + i;
			geometry_msgs::Pose temp;
			temp = bin_2_center;
			temp.position.x = bin_2_center.position.x + bin_2_x_axis.first*bin_2_id_relative_pose_map[id].first + bin_2_y_axis.first * bin_2_id_relative_pose_map[id].second;
			temp.position.y = bin_2_center.position.y + bin_2_x_axis.second*bin_2_id_relative_pose_map[id].first + bin_2_y_axis.second * bin_2_id_relative_pose_map[id].second; 
			//bin_2_id_pose_map[id] = temp;
			bin_id_pose_map["bin_2"][id] = temp;
			// ROS_WARN_STREAM("id_"<<id<<": ["<<bin_id_pose_map["bin_2"][id].position.x<<","<<bin_id_pose_map["bin_2"][id].position.y<<","<<bin_id_pose_map["bin_2"][id].position.z<<"]");
		}

	}
	for(int i = 0; i < 3; i ++){
		for(int j = 0; j < 3; j ++){
			int id = 3*i + j;
			geometry_msgs::Pose temp;
			temp = bin_1_center;
			temp.position.x = bin_1_center.position.x + bin_1_x_axis.first*bin_1_id_relative_pose_map[id].first + bin_1_y_axis.first * bin_1_id_relative_pose_map[id].second;
			temp.position.y = bin_1_center.position.y + bin_1_x_axis.second*bin_1_id_relative_pose_map[id].first + bin_1_y_axis.second * bin_1_id_relative_pose_map[id].second; 
			//bin_1_id_pose_map[id] = temp;
			bin_id_pose_map["bin_1"][id] = temp;
			// ROS_WARN_STREAM("id_"<<id<<": ["<<bin_id_pose_map["bin_1"][id].position.x<<","<<bin_id_pose_map["bin_1"][id].position.y<<","<<bin_id_pose_map["bin_1"][id].position.z<<"]");
		}

	}
*/
	char obj_topic[50];
    std::string obj_name;
    double obj_length = 0;
    double obj_width = 0;
    double z_length = 0;
    int num_objects = 0;
  
    
    if(nh.hasParam("/iiwa/objects/num_objects"))
      nh.getParam("/iiwa/objects/num_objects", num_objects);
    else
    {
      ROS_WARN_STREAM("Could not read the object config file...");
      exit(1);
    }
    for(int ii = 0; ii < num_objects; ii++)
    { 
      sprintf(obj_topic, "/objects/object_%d", ii+1);
      nh.getParam((std::string(obj_topic) + "/name").c_str(), obj_name);
      if(obj_name == target_object_name){
        nh.getParam((std::string(obj_topic) + "/x_dimension").c_str(), obj_length);
        nh.getParam((std::string(obj_topic) + "/y_dimension").c_str(), obj_width); 
        nh.getParam((std::string(obj_topic) + "/z_dimension").c_str(), z_length); 
        break;
      }
    }

	std::vector<geometry_msgs::Pose> grid_pose_vector_bin_1, grid_pose_vector_bin_2;
	geometry_msgs::Pose bin_1_center, bin_2_center;

	bin_2_center.position.x = 0.423;
	bin_2_center.position.y = 0.342;
	bin_2_center.position.z = 0.46;
	bin_2_center.orientation.x = -0.334696;
	bin_2_center.orientation.y = 0.94232;
	bin_2_center.orientation.z = -0.00172;
	bin_2_center.orientation.w = 0.00083;
	// bin_1_center.position.x = 0.4088;
	// bin_1_center.position.y = -0.329;
	#ifdef USE_DOVE
	bin_1_center.position.x = 0.4088  - 0.01 - 0.004;
	bin_1_center.position.y = -0.329 + 0.005 + 0.0025;
	#endif
	#ifdef USE_TOOTH
	bin_1_center.position.x = 0.4088  - 0.01 - 0.004 + 0.005*2;
	bin_1_center.position.y = -0.329 + 0.005 + 0.0025 + 0.008*2;
	#endif
	bin_1_center.position.z = 0.46;
	bin_1_center.orientation.x = 0.34554;
	bin_1_center.orientation.y = 0.93840;
	bin_1_center.orientation.z = -0.001849;
	bin_1_center.orientation.w = -0.000465;
	std::pair<double, double> bin_1_x_axis, bin_1_y_axis, bin_2_x_axis, bin_2_y_axis;
	bin_2_x_axis.first = 0.5;
	bin_2_x_axis.second = -0.866025404;
	bin_2_y_axis.first = 0.866025404;
	bin_2_y_axis.second = 0.5;
	bin_1_x_axis.first = -0.5;
	bin_1_x_axis.second = -0.866025404;
	bin_1_y_axis.first = 0.866025404;
	bin_1_y_axis.second = -0.5;
	double bin_2_x_length = 0.55;
	double bin_2_y_length = 0.37;
	double bin_1_x_length = 0.2921;
	double bin_1_y_length = 0.21082;
	
	std::map<int, std::pair<double, double>> bin_1_id_relative_pose_map, bin_2_id_relative_pose_map;

	int bin_1_x_num, bin_1_y_num, bin_1_total_num;
	bool is_align_with_bin_1 = true;
	bool is_align_with_bin_2 = true;
	bin_1_x_num = bin_1_x_length / obj_length;
	bin_1_y_num = bin_1_y_length / obj_width;
	bin_1_total_num = bin_1_x_num * bin_1_y_num;

	bin_1_x_num = bin_1_x_length / obj_width;
	bin_1_y_num = bin_1_y_length / obj_length;
	if(bin_1_x_num * bin_1_y_num > bin_1_total_num){
		bin_1_total_num = bin_1_x_num * bin_1_y_num;
		is_align_with_bin_1 = false;
	}else{
		bin_1_x_num = bin_1_x_length / obj_length;
		bin_1_y_num = bin_1_y_length / obj_width;
	}

	for(int j = 0; j < 4; j++){
		for(int i = 0; i < 3; i++){
			int id = 3*j + i;
			std::pair<double, double> relative_pose;
			relative_pose.first = bin_2_x_length / 5.0 * (j+1) - bin_2_x_length * 0.5; 
			relative_pose.second = (bin_2_y_length - 0.05)*0.5 - (bin_2_y_length-0.05) / 4.0 *(i+1); 
			bin_2_id_relative_pose_map[id] = relative_pose;
		}
	}

	for(int i = 0; i < bin_1_x_num; i++){
		for(int j = 0; j < bin_1_y_num; j++){
			int id = bin_1_y_num*i + j;
			std::pair<double, double> relative_pose;
			relative_pose.first = bin_1_x_length / (2*bin_1_x_num) * (2*i+1)  - bin_1_x_length * 0.5; 
			relative_pose.second = (bin_1_y_length) / (2*bin_1_y_num) *(2*j+1) - (bin_1_y_length)*0.5; 
			bin_1_id_relative_pose_map[id] = relative_pose;
		}
	}

	for(int j = 0; j < 4; j ++){
		for(int i = 0; i < 3; i ++){
			int id = 3*j + i;
			geometry_msgs::Pose temp;
			temp = bin_2_center;
			temp.position.x = bin_2_center.position.x + bin_2_x_axis.first*bin_2_id_relative_pose_map[id].first + bin_2_y_axis.first * bin_2_id_relative_pose_map[id].second;
			temp.position.y = bin_2_center.position.y + bin_2_x_axis.second*bin_2_id_relative_pose_map[id].first + bin_2_y_axis.second * bin_2_id_relative_pose_map[id].second; 
			//bin_2_id_pose_map[id] = temp;
			
			bin_id_pose_map["bin_2"][id] = temp;

			// ROS_WARN_STREAM("id_"<<id<<": ["<<bin_id_pose_map["bin_2"][id].position.x<<","<<bin_id_pose_map["bin_2"][id].position.y<<","<<bin_id_pose_map["bin_2"][id].position.z<<"]");
		}

	}
	for(int i = 0; i < bin_1_x_num; i ++){
		for(int j = 0; j < bin_1_y_num; j ++){
			int id = bin_1_y_num*i + j;
			geometry_msgs::Pose temp;
			temp = bin_1_center;
			temp.position.x = bin_1_center.position.x + bin_1_x_axis.first*bin_1_id_relative_pose_map[id].first + bin_1_y_axis.first * bin_1_id_relative_pose_map[id].second;
			temp.position.y = bin_1_center.position.y + bin_1_x_axis.second*bin_1_id_relative_pose_map[id].first + bin_1_y_axis.second * bin_1_id_relative_pose_map[id].second; 
			//bin_1_id_pose_map[id] = temp;
			if(is_align_with_bin_1){
				temp.orientation.x = 0;
				temp.orientation.y = 0;
				temp.orientation.z = 0.5;
				temp.orientation.w = 0.8660254;
			}else{
				temp.orientation.x = 0;
				temp.orientation.y = 0;
				temp.orientation.z = 0.9659258;
				temp.orientation.w = 0.258819;
			}
			bin_id_pose_map["bin_1"][id] = temp;

			ROS_WARN_STREAM("id_"<<id<<": ["<<bin_id_pose_map["bin_1"][id].position.x<<","<<bin_id_pose_map["bin_1"][id].position.y<<","<<bin_id_pose_map["bin_1"][id].position.z<<"]");
		}

	}

	ROS_ERROR_STREAM("Reading the object models...");
	read_object_models();


	ROS_ERROR_STREAM("Setting up the placement module...");
	return bin_1_total_num;
}


geometry_msgs::Pose  placement_module::get_grid_pose(std::string target_bin, int grid_id)
{
	auto ret = bin_id_pose_map[target_bin][grid_id];
	// int count = -1;
	// for(auto key: bin_id_pose_map[target_bin])
	// 	count++;


	// if(bin_counters[target_bin]+1>count)
	// 	bin_counters[target_bin] = 0;
	// else
	// 	bin_counters[target_bin]++;

	ret.position.z  = -0.18;

	return ret;
}



geometry_msgs::Pose  placement_module::get_target_pose(std::string target_bin)
{
	auto ret = bin_id_pose_map[target_bin][bin_counters[target_bin]];
	// int count = -1;
	// for(auto key: bin_id_pose_map[target_bin])
	// 	count++;


	// if(bin_counters[target_bin]+1>count)
	// 	bin_counters[target_bin] = 0;
	// else
	// 	bin_counters[target_bin]++;

	std::cout<<"\nPlacement module: "<<target_bin<<" ["<<bin_counters[target_bin]<<"] : "<<ret.position.x<<" "<<ret.position.y<<" "<<ret.position.z<<" ";
	ret.position.z -= 0.22;
	ret.position.z += (0.055)*layer;
	object_pose_on_grid = ret;
	object_pose_on_grid.position.z = -0.18 + 0.055*layer;


	ret.orientation.x = 0.34554;
	ret.orientation.y = 0.93840;
	ret.orientation.z = -0.001849;
	ret.orientation.w = -0.000465;
	return ret;
}





geometry_msgs::Pose  placement_module::get_target_pose(std::string label, geometry_msgs::Pose object_pose, std::string target_bin)
{
	auto ret = get_target_pose(target_bin);

	ROS_INFO_STREAM("Input object detection in bin "<<target_bin<<" for object "<<label<<" is  "<<object_pose.position.x<<" "<<object_pose.position.y<<" "<<object_pose.position.z<<" "<<object_pose.orientation.x<<" "<<object_pose.orientation.y<<" "<<object_pose.orientation.z<<" "<<object_pose.orientation.w<<" ");

	auto surfaces = surface_map[label];

	Eigen::Vector3f reference_direction(0,0,1);
	double max_dot_product = -10000;
	object_surface* top_facing_surface = NULL;

	for(auto surface: surfaces)
	{
		double dot_product = get_dot_product_of_surface_normal(object_pose, surface, reference_direction);
		// ROS_WARN_STREAM("For object: "<<label<<" CHECKING surface dot product "<<dot_product << " / "<<max_dot_product);
		if(dot_product>max_dot_product)
		{
			ROS_WARN_STREAM("For object: "<<label<<" more promising surface dot product "<<dot_product << " / "<<max_dot_product);
			top_facing_surface = surface;
			max_dot_product = dot_product;
		}
	}

	if(top_facing_surface==NULL)
	{
		ROS_ERROR_STREAM("EMPTY SURFACE");
	}
	auto primary_axis = top_facing_surface->primary_axis();


	Eigen::Quaternionf q;
	q.w() = object_pose.orientation.w;
	q.x() = object_pose.orientation.x;
	q.y() = object_pose.orientation.y;
	q.z() = object_pose.orientation.z;
	Eigen::Matrix3f rotMat;
	rotMat = q.toRotationMatrix();
	Eigen::Vector3f primary_axis_world = rotMat*primary_axis;
	primary_axis_world.normalize();

	double pi = 3.14159;
	auto axis_angle = atan(primary_axis_world[1]/(primary_axis_world[0]+0.00000001));
	if(axis_angle<0)
	{
		axis_angle = pi + axis_angle;
	}

	auto ref_angle = pi/2 + (50*pi/180);

	if(target_bin=="bin_2")
		ref_angle = pi*3/2 - (50*pi/180);


	auto deviation = ref_angle - axis_angle;



	// auto target_rotation_about_z = ref_angle - deviation;
	auto target_rotation_about_z = deviation;
	if(target_rotation_about_z<0)
	{
		// target_rotation_about_z = -(pi - target_rotation_about_z);
		target_rotation_about_z = (2*pi + target_rotation_about_z);
	}
	if(target_rotation_about_z>pi)
	{
		target_rotation_about_z-=pi;
	}
	if(target_rotation_about_z>pi/2)
	{
		target_rotation_about_z = -(pi - target_rotation_about_z);
	}



	computed_rotation_about_z = target_rotation_about_z;
	computed_alignment = axis_angle;
	//Find quaterion for (0,0,-1), and rotation of target_rotation_about_z
	Eigen::Quaternionf grid_q;
	grid_q.w() = ret.orientation.w;
	grid_q.x() = ret.orientation.x;
	grid_q.y() = ret.orientation.y;
	grid_q.z() = ret.orientation.z;
	Eigen::Matrix3f grid_m = grid_q.toRotationMatrix();

	ROS_WARN_STREAM("The current rotation matrix is \n"<<grid_m);
	ROS_WARN_STREAM("The Rotation is about primary axis \n"<<primary_axis_world);
	ROS_WARN_STREAM("DEVIATION: "<<axis_angle*180/pi<<" vs "<<ref_angle*180/pi<<" = "<<deviation*180/pi);
	ROS_ERROR_STREAM("The target rotation about z is "<<target_rotation_about_z*180/pi);
	// target_rotation_about_z = pi/2;
	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
	  * Eigen::AngleAxisf(0,  Eigen::Vector3f::UnitY())
	  * Eigen::AngleAxisf(target_rotation_about_z, Eigen::Vector3f::UnitZ());



	Eigen::Quaternionf q_grid(m*grid_m);
	// Eigen::Quaternionf q_grid(grid_m*m);
	ret.orientation.w = q_grid.w();
	ret.orientation.x = q_grid.x();
	ret.orientation.y = q_grid.y();
	ret.orientation.z = q_grid.z();


	return ret;
}


std::pair<geometry_msgs::Pose, geometry_msgs::Pose> placement_module::get_target_pose(std::string label, geometry_msgs::Pose object_pose, geometry_msgs::Pose grasp_pose, std::string target_bin)
{
	auto ret = get_target_pose(label, object_pose, target_bin);
	std::pair<geometry_msgs::Pose, geometry_msgs::Pose> returned_pair;
	
	double target_rotation_about_z = computed_rotation_about_z;
	double object_alignment = computed_alignment;


	auto surfaces = surface_map[label];

	Eigen::Vector3f reference_direction(0,0,1);
	double max_dot_product = -10000;
	object_surface* top_facing_surface = NULL;

	for(auto surface: surfaces)
	{
		double dot_product = get_dot_product_of_surface_normal(object_pose, surface, reference_direction);
		// ROS_WARN_STREAM("For object: "<<label<<" CHECKING surface dot product "<<dot_product << " / "<<max_dot_product);
		if(dot_product>max_dot_product)
		{
			// ROS_WARN_STREAM("For object: "<<label<<" more promising surface dot product "<<dot_product << " / "<<max_dot_product);
			top_facing_surface = surface;
			max_dot_product = dot_product;
		}
	}

	if(top_facing_surface==NULL)
	{
		ROS_ERROR_STREAM("EMPTY SURFACE");
	}
	else
	{
		ROS_ERROR_STREAM("\nTOP SURFACE: ");
		top_facing_surface->print();
	}
	auto surface_center = top_facing_surface->center_point();

	ROS_INFO_STREAM("Surface center: "<<surface_center[0]<<"  "<<surface_center[1]<<"  "<<surface_center[2]<<"  ");


	Eigen::Quaternionf q;
	q.w() = object_pose.orientation.w;
	q.x() = object_pose.orientation.x;
	q.y() = object_pose.orientation.y;
	q.z() = object_pose.orientation.z;
	Eigen::Matrix3f rotMat;
	rotMat = q.toRotationMatrix();
	Eigen::Vector3f surface_center_world = rotMat*surface_center;
	surface_center_world[0]+=object_pose.position.x;
	surface_center_world[1]+=object_pose.position.y;
	surface_center_world[2]+=object_pose.position.z;
	ROS_INFO_STREAM("Surface center world: "<<surface_center_world[0]<<"  "<<surface_center_world[1]<<"  "<<surface_center_world[2]<<"  ");

	Eigen::Vector3f grasp_vec(grasp_pose.position.x,grasp_pose.position.y,grasp_pose.position.z);
	ROS_WARN_STREAM("GRASP VECTOR: "<<grasp_vec[0]<<"  "<<grasp_vec[1]<<"  "<<grasp_vec[2]<<"  ");
	double offset = get_distance(grasp_vec, surface_center_world);

	double pi = 3.14159;
	double bin_to_bin_change = -80*pi/180;
	if(target_bin=="bin_2")
		bin_to_bin_change = -bin_to_bin_change;

	double offset_alignment = atan((grasp_vec[1]-surface_center_world[1])/((grasp_vec[0]-surface_center_world[0])+0.00000001));

	double offset_x = offset*cos( offset_alignment + bin_to_bin_change + target_rotation_about_z ); 
	double offset_y = offset*sin( offset_alignment + bin_to_bin_change + target_rotation_about_z );

	ROS_WARN_STREAM("Offset: "<<offset<<" ||| -> "<<offset_x<<"  "<<offset_y );

	ret.position.x -= offset_x;
	ret.position.y -= offset_y;

	// int xxx;
	// std::cin>>xxx;
	returned_pair.first = ret;
	returned_pair.second = object_pose_on_grid;

	// return ret;
	return returned_pair;

}

bool placement_module::is_top_surface_graspable(std::string label, geometry_msgs::Pose current_object)
{
	auto surfaces = surface_map[label];

	Eigen::Vector3f reference_direction(0,0,1);
	double max_dot_product = -10000;
	object_surface* top_facing_surface = NULL;

	for(auto surface: surfaces)
	{
		double dot_product = get_dot_product_of_surface_normal(current_object, surface, reference_direction);
		// ROS_WARN_STREAM("For object: "<<label<<" CHECKING surface dot product "<<dot_product << " / "<<max_dot_product);
		if(dot_product>max_dot_product)
		{
			// ROS_WARN_STREAM("For object: "<<label<<" more promising surface dot product "<<dot_product << " / "<<max_dot_product);
			top_facing_surface = surface;
			max_dot_product = dot_product;
		}
	}

	if(top_facing_surface==NULL)
	{
		ROS_ERROR_STREAM("EMPTY SURFACE");
	}
	else
	{
		ROS_ERROR_STREAM("\nTOP SURFACE: ");
		top_facing_surface->print();
	}

	return top_facing_surface->graspable;
}

std::pair<double, object_surface* >
 placement_module::check_primary_axis(std::string label, geometry_msgs::Pose object_pose, bool ret_surface)
{
	auto surfaces = surface_map[label];

	Eigen::Vector3f reference_direction(0,0,1);
	double max_dot_product = -10000;
	object_surface* top_facing_surface = NULL;

	for(auto surface: surfaces)
	{
		double dot_product = get_dot_product_of_surface_normal(object_pose, surface, reference_direction);
		// ROS_WARN_STREAM("For object: "<<label<<" CHECKING surface dot product "<<dot_product << " / "<<max_dot_product);
		if(dot_product>max_dot_product)
		{
			// ROS_WARN_STREAM("For object: "<<label<<" more promising surface dot product "<<dot_product << " / "<<max_dot_product);
			top_facing_surface = surface;
			max_dot_product = dot_product;
		}
	}

	if(top_facing_surface==NULL)
	{
		ROS_ERROR_STREAM("EMPTY SURFACE");
	}
	else
	{
		ROS_ERROR_STREAM("\nTOP SURFACE: ");
		top_facing_surface->print();
	}
	auto primary_axis = top_facing_surface->primary_axis();


	Eigen::Quaternionf q;
	q.w() = object_pose.orientation.w;
	q.x() = object_pose.orientation.x;
	q.y() = object_pose.orientation.y;
	q.z() = object_pose.orientation.z;
	Eigen::Matrix3f rotMat;
	rotMat = q.toRotationMatrix();

	Eigen::Vector3f primary_axis_world = rotMat*primary_axis;
	primary_axis_world.normalize();

	//Compute rotation
	//########################################################################
	//########################################################################
	double pi = 3.14159;
	auto axis_angle = atan(primary_axis_world[1]/(primary_axis_world[0]+0.00000001));


	ROS_ERROR_STREAM("The target orientation of the object at \n"<<object_pose << " \n is aligning the primary axis\n "<<primary_axis_world<<" \n along the angle \n "<<axis_angle);

	return std::make_pair(axis_angle, top_facing_surface);
}

double placement_module::check_primary_axis(std::string label, geometry_msgs::Pose object_pose)
{
	std::pair <double, object_surface* > ret = check_primary_axis(label, object_pose, true);

	return ret.first;
}



geometry_msgs::Pose placement_module::get_adjusted_ee_pose(std::string label, std::string target_bin, geometry_msgs::Pose current_ee, geometry_msgs::Pose current_object, geometry_msgs::Pose target_object)
{
	double target_alignment_angle = check_primary_axis(label, target_object);

	auto surfaces = surface_map[label];

	Eigen::Vector3f reference_direction(0,0,1);
	double max_dot_product = -10000;
	object_surface* top_facing_surface = NULL;

	for(auto surface: surfaces)
	{
		double dot_product = get_dot_product_of_surface_normal(current_object, surface, reference_direction);
		// ROS_WARN_STREAM("For object: "<<label<<" CHECKING surface dot product "<<dot_product << " / "<<max_dot_product);
		if(dot_product>max_dot_product)
		{
			// ROS_WARN_STREAM("For object: "<<label<<" more promising surface dot product "<<dot_product << " / "<<max_dot_product);
			top_facing_surface = surface;
			max_dot_product = dot_product;
		}
	}

	if(top_facing_surface==NULL)
	{
		ROS_ERROR_STREAM("EMPTY SURFACE");
	}
	else
	{
		ROS_ERROR_STREAM("\nTOP SURFACE: ");
		top_facing_surface->print();
	}
	auto surface_center = top_facing_surface->center_point();
	auto primary_axis = top_facing_surface->primary_axis();

	ROS_INFO_STREAM("Surface center: "<<surface_center[0]<<"  "<<surface_center[1]<<"  "<<surface_center[2]<<"  ");


	Eigen::Quaternionf q;
	q.w() = current_object.orientation.w;
	q.x() = current_object.orientation.x;
	q.y() = current_object.orientation.y;
	q.z() = current_object.orientation.z;
	Eigen::Matrix3f rotMat;
	rotMat = q.toRotationMatrix();


	Eigen::Vector3f surface_center_world = rotMat*surface_center;
	surface_center_world[0]+=current_object.position.x;
	surface_center_world[1]+=current_object.position.y;
	surface_center_world[2]+=current_object.position.z;

	
	ROS_INFO_STREAM("Surface center world: "<<surface_center_world[0]<<"  "<<surface_center_world[1]<<"  "<<surface_center_world[2]<<"  ");

	Eigen::Vector3f primary_axis_world = rotMat*primary_axis;
	primary_axis_world.normalize();


	double offset_x = target_object.position.x - surface_center_world[0];
	double offset_y = target_object.position.y - surface_center_world[1];





	//Compute rotation
	//########################################################################
	//########################################################################
	double pi = 3.14159;
	auto axis_angle = atan(primary_axis_world[1]/(primary_axis_world[0]+0.00000001));
	if(axis_angle<0)
	{
		axis_angle = pi + axis_angle;
	}

	// auto ref_angle = pi/2 + (30*pi/180);
	// //Logic gets flipped
	// if(target_bin=="bin_1")
	// 	ref_angle = pi*3/2 - (30*pi/180);

	auto ref_angle = target_alignment_angle;


	auto deviation = ref_angle - axis_angle;
	// auto target_rotation_about_z = ref_angle - deviation;
	auto target_rotation_about_z = deviation;
	if(target_rotation_about_z<0)
	{
		// target_rotation_about_z = -(pi - target_rotation_about_z);
		target_rotation_about_z = (2*pi + target_rotation_about_z);
	}
	if(target_rotation_about_z>pi)
	{
		target_rotation_about_z-=pi;
	}
	if(target_rotation_about_z>pi/2)
	{
		target_rotation_about_z = -(pi - target_rotation_about_z);
	}
	//########################################################################
	//########################################################################



	//Find quaterion for (0,0,-1), and rotation of target_rotation_about_z
	Eigen::Quaternionf grid_q;
	grid_q.w() = current_ee.orientation.w;
	grid_q.x() = current_ee.orientation.x;
	grid_q.y() = current_ee.orientation.y;
	grid_q.z() = current_ee.orientation.z;
	Eigen::Matrix3f grid_m = grid_q.toRotationMatrix();
	ROS_ERROR_STREAM("\n########################\n########################\nFinal adjustment before drop.\n########################\n########################\n");
	ROS_WARN_STREAM("The current rotation matrix is \n"<<grid_m);
	ROS_WARN_STREAM("The Rotation is about primary axis \n"<<primary_axis_world);
	ROS_WARN_STREAM("DEVIATION: "<<axis_angle*180/pi<<" vs "<<ref_angle*180/pi<<" = "<<deviation*180/pi);
	ROS_ERROR_STREAM("The target rotation about z is "<<target_rotation_about_z*180/pi);
	// target_rotation_about_z = pi/2;
	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
	  * Eigen::AngleAxisf(0,  Eigen::Vector3f::UnitY())
	  * Eigen::AngleAxisf(target_rotation_about_z, Eigen::Vector3f::UnitZ());

	Eigen::Quaternionf q_grid(m*grid_m);
	// Eigen::Quaternionf q_grid(grid_m*m);


	geometry_msgs::Pose adjusted_pose = current_ee;
	adjusted_pose.orientation.w = q_grid.w();
	adjusted_pose.orientation.x = q_grid.x();
	adjusted_pose.orientation.y = q_grid.y();
	adjusted_pose.orientation.z = q_grid.z();


	adjusted_pose.position.x+=offset_x;
	adjusted_pose.position.y+=offset_y;

	return adjusted_pose;

}








std::pair< geometry_msgs::Pose, std::vector< std::vector<geometry_msgs::Pose> > > placement_module::get_adjusted_regrasp_pose(std::string label, std::string source_bin, geometry_msgs::Pose current_ee, geometry_msgs::Pose current_object, geometry_msgs::Pose target_plane_point, double target_alignment_angle)
{
	std::pair< geometry_msgs::Pose, std::vector< std::vector<geometry_msgs::Pose> > > return_maneuvers;

	auto surfaces = surface_map[label];

	Eigen::Vector3f reference_direction(0,0,-1);
	double max_dot_product = -10000;
	object_surface* bottom_facing_normal = NULL;

	for(auto surface: surfaces)
	{
		double dot_product = get_dot_product_of_surface_normal(current_object, surface, reference_direction);
		// ROS_WARN_STREAM("For object: "<<label<<" CHECKING surface dot product "<<dot_product << " / "<<max_dot_product);
		if(dot_product>max_dot_product)
		{
			// ROS_WARN_STREAM("For object: "<<label<<" more promising surface dot product "<<dot_product << " / "<<max_dot_product);
			bottom_facing_normal = surface;
			max_dot_product = dot_product;
		}
	}

	if(bottom_facing_normal==NULL)
	{
		ROS_ERROR_STREAM("EMPTY SURFACE");
	}
	else
	{
		ROS_ERROR_STREAM("\nBOTTOM SURFACE: ");
		bottom_facing_normal->print();
	}
	auto surface_center = bottom_facing_normal->center_point();
	auto primary_axis = bottom_facing_normal->primary_axis();
	auto secondary_axis = bottom_facing_normal->secondary_axis();
	auto surface_normal = bottom_facing_normal->surface_normal();
	auto dimensions = bottom_facing_normal->dimensions;

	ROS_INFO_STREAM("Surface center: "<<surface_center[0]<<"  "<<surface_center[1]<<"  "<<surface_center[2]<<"  ");


	Eigen::Quaternionf q;
	q.w() = current_object.orientation.w;
	q.x() = current_object.orientation.x;
	q.y() = current_object.orientation.y;
	q.z() = current_object.orientation.z;
	Eigen::Matrix3f rotMat;
	rotMat = q.toRotationMatrix();


	Eigen::Vector3f surface_center_world = rotMat*surface_center;
	surface_center_world[0]+=current_object.position.x;
	surface_center_world[1]+=current_object.position.y;
	surface_center_world[2]+=current_object.position.z;


	Eigen::Vector3f offsets(  target_plane_point.position.x-surface_center_world[0],
							  target_plane_point.position.y-surface_center_world[1],
							  target_plane_point.position.z-surface_center_world[2]
		);




	Eigen::Vector3f primary_axis_world = rotMat*primary_axis;
	primary_axis_world.normalize();

	//Compute rotation
	//########################################################################
	//########################################################################
	double pi = 3.14159;
	auto axis_angle = atan(primary_axis_world[1]/(primary_axis_world[0]+0.00000001));
	if(axis_angle<0)
	{
		axis_angle = pi + axis_angle;
	}

	auto ref_angle = target_alignment_angle*pi/180;


	auto deviation = ref_angle - axis_angle;
	auto target_rotation_about_z = deviation;
	
	//Shorter arcs including symmetry	
	if(target_rotation_about_z<0)
	{
		// target_rotation_about_z = -(pi - target_rotation_about_z);
		target_rotation_about_z = (2*pi + target_rotation_about_z);
	}
	if(target_rotation_about_z>pi)
	{
		target_rotation_about_z-=pi;
	}
	if(target_rotation_about_z>pi/2)
	{
		target_rotation_about_z = -(pi - target_rotation_about_z);
	}
	//########################################################################
	//########################################################################



	//Find quaterion for (0,0,-1), and rotation of target_rotation_about_z
	Eigen::Quaternionf grid_q;
	grid_q.w() = current_ee.orientation.w;
	grid_q.x() = current_ee.orientation.x;
	grid_q.y() = current_ee.orientation.y;
	grid_q.z() = current_ee.orientation.z;
	Eigen::Matrix3f current_ee_rot = grid_q.toRotationMatrix();
	ROS_ERROR_STREAM("\n########################\n########################\nFinal adjustment before drop.\n########################\n########################\n");
	ROS_WARN_STREAM("The current rotation matrix is \n"<<current_ee_rot);
	ROS_WARN_STREAM("The Rotation is about primary axis \n"<<primary_axis_world);
	ROS_WARN_STREAM("DEVIATION: "<<axis_angle*180/pi<<" vs "<<ref_angle*180/pi<<" = "<<deviation*180/pi);
	ROS_ERROR_STREAM("The target rotation about z is "<<target_rotation_about_z*180/pi);
	ROS_INFO_STREAM("OFFSETS \n"<<offsets);
	// target_rotation_about_z = pi/2;
	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
	  * Eigen::AngleAxisf(0,  Eigen::Vector3f::UnitY())
	  * Eigen::AngleAxisf(target_rotation_about_z, Eigen::Vector3f::UnitZ());

	Eigen::Quaternionf q_grid(m*current_ee_rot);


	geometry_msgs::Pose adjusted_pose = current_ee;
	adjusted_pose.orientation.w = q_grid.w();
	adjusted_pose.orientation.x = q_grid.x();
	adjusted_pose.orientation.y = q_grid.y();
	adjusted_pose.orientation.z = q_grid.z();


	adjusted_pose.position.x+=offsets[0];
	adjusted_pose.position.y+=offsets[1];
	adjusted_pose.position.z+=offsets[2];
	adjusted_pose.position.z+=0.1;
	


	
	return_maneuvers.first = adjusted_pose;

	adjusted_pose.position.z-=0.1;
	adjusted_pose.position.z-=0.015;



	Eigen::Matrix3f rotated_obj = m*rotMat;

	ROS_ERROR_STREAM("ROt Obj Mat: \n"<<rotated_obj);
	ROS_ERROR_STREAM("Secondary Axis: \n"<<secondary_axis);
	ROS_ERROR_STREAM("Secondary Axis DIM: "<<dimensions[1]);
	Eigen::Vector3f global_secondary_axis = rotated_obj*secondary_axis;
	global_secondary_axis.normalize();
	global_secondary_axis = global_secondary_axis * (0.05 + (1.2*dimensions[1]));
	// global_secondary_axis[0]=global_secondary_axis[0]*dimensions[1];
	// global_secondary_axis[1]=global_secondary_axis[1]*dimensions[1];
	// global_secondary_axis[2]=global_secondary_axis[2]*dimensions[1];

	ROS_WARN_STREAM("SECONDARY AXIS DRAG: \n"<<global_secondary_axis);


	auto lateral_drag = adjusted_pose;
	lateral_drag.position.x+=global_secondary_axis[0];
	lateral_drag.position.y+=global_secondary_axis[1];
	lateral_drag.position.z-=0.005;

	Eigen::Vector3f global_primary_axis = rotated_obj*primary_axis;
	global_primary_axis.normalize();


	Eigen::Vector3f global_object_center(current_object.position.x,current_object.position.y,current_object.position.z);
	Eigen::Vector3f global_surface_center = rotated_obj*surface_center;




	return_maneuvers.second.push_back({adjusted_pose,lateral_drag});


	//Tilting
	auto dist_from_center = get_distance(global_object_center, global_surface_center);
	auto short_axis = dimensions[1]/2;

	auto tilt_angle = atan(short_axis/(dist_from_center+0.00000001));
	Eigen::Vector3f vec1 = rotated_obj*(-surface_normal);
	vec1.normalize();
	Eigen::Vector3f vec2 = rotated_obj*(secondary_axis);
	vec2.normalize();
	auto tilt_axis =  ( vec1 ).cross( vec2 );
	Eigen::Matrix3f m_tilt (Eigen::AngleAxisf(tilt_angle, tilt_axis));
	Eigen::Quaternionf q_tilt(m_tilt*current_ee_rot);

	auto tilted_pose = adjusted_pose;
	tilted_pose.orientation.w = q_tilt.w();
	tilted_pose.orientation.x = q_tilt.x();
	tilted_pose.orientation.y = q_tilt.y();
	tilted_pose.orientation.z = q_tilt.z();

	Eigen::Vector3f xy_tilt_offsets = ( vec2 )*(0.38*cos(tilt_angle));
	auto z_offset = -0.38*sin(tilt_angle);

	tilted_pose.position.x+=xy_tilt_offsets[0];
	tilted_pose.position.y+=xy_tilt_offsets[1];
	tilted_pose.position.z+=z_offset;

	ROS_WARN_STREAM("###########TILT############");
	ROS_WARN_STREAM("Angle: "<<tilt_angle<<"\n about \n"<<tilt_axis<<"\n offset by \n"<<xy_tilt_offsets<<"\n and "<<z_offset<<" \n TILTED POSE:\n");
	ROS_WARN_STREAM(tilted_pose);
	ROS_WARN_STREAM("###########TILT############");
	
	// return_maneuvers.second.push_back({tilted_pose});


	return return_maneuvers;
}







geometry_msgs::Pose placement_module::get_adjusted_aligned_ee_pose(std::string label, std::string source_bin, geometry_msgs::Pose current_ee, geometry_msgs::Pose current_object, double target_alignment_angle)
{

	auto surfaces = surface_map[label];

	Eigen::Vector3f reference_direction(0,0,1);
	double max_dot_product = -10000;
	object_surface* top_facing_normal = NULL;

	for(auto surface: surfaces)
	{
		double dot_product = get_dot_product_of_surface_normal(current_object, surface, reference_direction);
		// ROS_WARN_STREAM("For object: "<<label<<" CHECKING surface dot product "<<dot_product << " / "<<max_dot_product);
		if(dot_product>max_dot_product)
		{
			// ROS_WARN_STREAM("For object: "<<label<<" more promising surface dot product "<<dot_product << " / "<<max_dot_product);
			top_facing_normal = surface;
			max_dot_product = dot_product;
		}
	}

	if(top_facing_normal==NULL)
	{
		ROS_ERROR_STREAM("EMPTY SURFACE");
	}
	else
	{
		ROS_ERROR_STREAM("\nBOTTOM SURFACE: ");
		top_facing_normal->print();
	}
	auto surface_center = top_facing_normal->center_point();
	auto primary_axis = top_facing_normal->primary_axis();
	auto secondary_axis = top_facing_normal->secondary_axis();
	auto surface_normal = top_facing_normal->surface_normal();
	auto dimensions = top_facing_normal->dimensions;

	ROS_INFO_STREAM("Surface center: "<<surface_center[0]<<"  "<<surface_center[1]<<"  "<<surface_center[2]<<"  ");


	Eigen::Quaternionf q;
	q.w() = current_object.orientation.w;
	q.x() = current_object.orientation.x;
	q.y() = current_object.orientation.y;
	q.z() = current_object.orientation.z;
	Eigen::Matrix3f rotMat;
	rotMat = q.toRotationMatrix();


	Eigen::Vector3f surface_center_world = rotMat*surface_center;
	surface_center_world[0]+=current_object.position.x;
	surface_center_world[1]+=current_object.position.y;
	surface_center_world[2]+=current_object.position.z;


	Eigen::Vector3f offsets(  0,
							  0,
							  0
		);




	Eigen::Vector3f primary_axis_world = rotMat*primary_axis;
	primary_axis_world.normalize();

	//Compute rotation
	//########################################################################
	//########################################################################
	double pi = 3.14159;
	auto axis_angle = atan(primary_axis_world[1]/(primary_axis_world[0]+0.00000001));
	if(axis_angle<0)
	{
		axis_angle = pi + axis_angle;
	}

	auto ref_angle = target_alignment_angle*pi/180;


	auto deviation = ref_angle - axis_angle;
	auto target_rotation_about_z = deviation;
	
	//Shorter arcs including symmetry	
	if(target_rotation_about_z<0)
	{
		// target_rotation_about_z = -(pi - target_rotation_about_z);
		target_rotation_about_z = (2*pi + target_rotation_about_z);
	}
	if(target_rotation_about_z>pi)
	{
		target_rotation_about_z-=pi;
	}
	if(target_rotation_about_z>pi/2)
	{
		target_rotation_about_z = -(pi - target_rotation_about_z);
	}
	//########################################################################
	//########################################################################



	//Find quaterion for (0,0,-1), and rotation of target_rotation_about_z
	Eigen::Quaternionf grid_q;
	grid_q.w() = current_ee.orientation.w;
	grid_q.x() = current_ee.orientation.x;
	grid_q.y() = current_ee.orientation.y;
	grid_q.z() = current_ee.orientation.z;
	Eigen::Matrix3f current_ee_rot = grid_q.toRotationMatrix();
	ROS_ERROR_STREAM("\n########################\n########################\nFinal adjustment before drop.\n########################\n########################\n");
	ROS_WARN_STREAM("The current rotation matrix is \n"<<current_ee_rot);
	ROS_WARN_STREAM("The Rotation is about primary axis \n"<<primary_axis_world);
	ROS_WARN_STREAM("DEVIATION: "<<axis_angle*180/pi<<" vs "<<ref_angle*180/pi<<" = "<<deviation*180/pi);
	ROS_ERROR_STREAM("The target rotation about z is "<<target_rotation_about_z*180/pi);
	ROS_INFO_STREAM("OFFSETS \n"<<offsets);
	// target_rotation_about_z = pi/2;
	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
	  * Eigen::AngleAxisf(0,  Eigen::Vector3f::UnitY())
	  * Eigen::AngleAxisf(target_rotation_about_z, Eigen::Vector3f::UnitZ());

	Eigen::Quaternionf q_grid(m*current_ee_rot);


	geometry_msgs::Pose adjusted_pose = current_ee;
	adjusted_pose.orientation.w = q_grid.w();
	adjusted_pose.orientation.x = q_grid.x();
	adjusted_pose.orientation.y = q_grid.y();
	adjusted_pose.orientation.z = q_grid.z();


	adjusted_pose.position.x+=offsets[0];
	adjusted_pose.position.y+=offsets[1];
	adjusted_pose.position.z+=offsets[2];

	return adjusted_pose;

}







double placement_module::get_distance(Eigen::Vector3f p1, Eigen::Vector3f p2)
{
	return std::sqrt( (p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]) + (p1[2]-p2[2])*(p1[2]-p2[2])  );
}


double placement_module::get_dot_product_of_surface_normal(geometry_msgs::Pose object_pose, object_surface* surface, Eigen::Vector3f reference_direction)
{
	double dot_product;
	//Transform surface->normal to world frame
	Eigen::Quaternionf q;
	q.w() = object_pose.orientation.w;
	q.x() = object_pose.orientation.x;
	q.y() = object_pose.orientation.y;
	q.z() = object_pose.orientation.z;
	Eigen::Matrix3f rotMat;
	rotMat = q.toRotationMatrix();

	Eigen::Vector3f surface_normal_world_frame = rotMat*surface->surface_normal();
	// ROS_WARN_STREAM("Comparing the normals..."<<reference_direction<<" and "<<surface_normal_world_frame<<"  from  "<<surface->surface_normal());
	dot_product = surface_normal_world_frame.dot(reference_direction);
	
	//Normalize surface normal
	//Normalize reference direction
	//Calculate the dot product

	return dot_product;
}























void placement_module::extract_surfaces_cuboid(std::string obj_name, pcl::PointCloud<pcl::PointNormal>::Ptr pcl_model) {

    pcl::MomentOfInertiaEstimation <pcl::PointNormal> feature_extractor;
    feature_extractor.setInputCloud (pcl_model);
    feature_extractor.compute ();

    pcl::PointNormal min_point_OBB;
    pcl::PointNormal max_point_OBB;
    pcl::PointNormal position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

  	feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  	
  	std::cout << "min_point_OBB" << min_point_OBB << std::endl;
  	std::cout << "max_point_OBB" << max_point_OBB << std::endl;
  	std::cout << "position_OBB" << position_OBB << std::endl;


  	//Extract surfaces
  	//Surface -X

  	Eigen::Vector3f min_point_OBB_vec(min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
  	Eigen::Vector3f max_point_OBB_vec(max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
  	std::vector<Eigen::Vector3f> bounds = {min_point_OBB_vec, max_point_OBB_vec};
  	Eigen::Vector3f position_OBB_vec(position_OBB.x, position_OBB.y, position_OBB.z);

  	//Get all the 6 surfaces of the cuboid
  	surface_map[obj_name].push_back(  get_surface(position_OBB_vec, bounds, 0, 0, {1,2}, rotational_matrix_OBB) );
  	surface_map[obj_name].push_back(  get_surface(position_OBB_vec, bounds, 0, 1, {1,2}, rotational_matrix_OBB) );
  	surface_map[obj_name].push_back(  get_surface(position_OBB_vec, bounds, 1, 0, {0,2}, rotational_matrix_OBB) );
  	surface_map[obj_name].push_back(  get_surface(position_OBB_vec, bounds, 1, 1, {0,2}, rotational_matrix_OBB) );
  	surface_map[obj_name].push_back(  get_surface(position_OBB_vec, bounds, 2, 0, {1,0}, rotational_matrix_OBB) );
  	surface_map[obj_name].push_back(  get_surface(position_OBB_vec, bounds, 2, 1, {1,0}, rotational_matrix_OBB) );


  	//Visualization
  	
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	// viewer->setBackgroundColor (0, 0, 0);
	// viewer->addCoordinateSystem (0.01);
	// viewer->initCameraParameters ();
	// viewer->addPointCloud<pcl::PointNormal> (pcl_model, "sample cloud");
	// Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    // Eigen::Quaternionf quat (rotational_matrix_OBB);
	// viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, 
	// 					max_point_OBB.z - min_point_OBB.z, "OBB");

	// int surface_index = 0;
	// for (auto surface_it: surface_map[obj_name]) {
	// 	pcl::PointXYZ center(surface_it->center[0], surface_it->center[1], surface_it->center[2]);

	// 	std::cout << "surface_index: " << surface_index << ", dimensions: " << surface_it->dimensions << std::endl;

	// 	Eigen::Vector3f extreme_pt_x = (surface_it->center + surface_it->rotation.col(0)*surface_it->dimensions[0]/2);
	// 	Eigen::Vector3f extreme_pt_y = (surface_it->center + surface_it->rotation.col(1)*surface_it->dimensions[1]/2);
	// 	Eigen::Vector3f extreme_pt_z = (surface_it->center + surface_it->rotation.col(2)*((position_OBB_vec - surface_it->center).norm()));

	// 	pcl::PointXYZ extreme_x(extreme_pt_x[0], extreme_pt_x[1], extreme_pt_x[2]);
	// 	pcl::PointXYZ extreme_y(extreme_pt_y[0], extreme_pt_y[1], extreme_pt_y[2]);
	// 	pcl::PointXYZ extreme_z(extreme_pt_z[0], extreme_pt_z[1], extreme_pt_z[2]);
	// 	viewer->addLine (center, extreme_x, 1.0f, 0.0f, 0.0f, "x-axis" + std::to_string(surface_index));
	// 	viewer->addLine (center, extreme_y, 0.0f, 1.0f, 0.0f, "y-axis" + std::to_string(surface_index));
	// 	viewer->addLine (center, extreme_z, 0.0f, 0.0f, 1.0f, "z-axis" + std::to_string(surface_index));
	// 	surface_index++;
	// }

	// while(!viewer->wasStopped()) {
	//     viewer->spinOnce (100);
	//     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    //}

}

object_surface* placement_module::get_surface(Eigen::Vector3f& position_OBB_vec, std::vector<Eigen::Vector3f>& bounds, 
	int surface_axis, int surface_direction, std::vector<int> other_axes, Eigen::Matrix3f &rotational_matrix_OBB)
{	

	object_surface* surface_ptr = new object_surface();
  	Eigen::Vector3f surface_normal(0, 0, 0);
  	surface_normal[surface_axis] = bounds[surface_direction][surface_axis];
  	// auto surface_center = position_OBB_vec + surface_normal;
  	auto surface_center = surface_normal;

  	std::vector<Eigen::Vector3f> corners;
	auto other_axis_1 = other_axes[0];
  	auto other_axis_2 = other_axes[1];

  	for(auto direction_1 : {0,1})
  	{
  		for(auto direction_2 : {0,1})
  		{
  			Eigen::Vector3f corner(0,0,0);
  			corner[other_axis_1] = bounds[direction_1][other_axis_1];
  			corner[other_axis_2] = bounds[direction_2][other_axis_2];
  			corner = surface_center + corner;
  			corners.push_back(corner);
  		}	
  	}

  	std::cout<<"Surface \n"<< surface_center<<" | normal: \n" << surface_normal <<   "\n";
  	for(auto corner: corners)
  	{
  		std::cout<<"Corner: \n"<<corner<<"\n";
  	}

  	auto dimension_1 = std::abs(bounds[1][other_axis_1]) + std::abs(bounds[0][other_axis_1]);
  	auto dimension_2 = std::abs(bounds[1][other_axis_2]) + std::abs(bounds[0][other_axis_2]);

  	std::cout<<"Dimensions: "<<dimension_1<<" and "<<dimension_2<<"\n";


  	Eigen::Vector3f primary_axis_vec(0,0,0);
  	Eigen::Vector3f secondary_axis_vec(0,0,0);

  	if(dimension_1 > dimension_2)
  	{
  		primary_axis_vec[other_axes[0]] = dimension_1;
  		secondary_axis_vec[other_axes[1]] = dimension_2;
  	}
  	else
  	{
  		primary_axis_vec[other_axes[1]] = dimension_2;
  		secondary_axis_vec[other_axes[0]] = dimension_1;
  	}

  	// surface_ptr->center = rotational_matrix_OBB*surface_center;
  	surface_ptr->center = position_OBB_vec + rotational_matrix_OBB*surface_center;
  	surface_ptr->dimensions[0] = primary_axis_vec.norm();
  	surface_ptr->dimensions[1] = secondary_axis_vec.norm();

  	Eigen::Matrix3f box_rotation;
  	box_rotation.col(0) = primary_axis_vec.normalized();
  	box_rotation.col(1) = secondary_axis_vec.normalized();
  	box_rotation.col(2) = surface_normal.normalized();

  	surface_ptr->rotation = rotational_matrix_OBB*box_rotation;
  	// box_rotation = box_rotation.inverse().eval();
  	// surface_ptr->rotation = rotational_matrix_OBB*box_rotation;
  	ROS_ERROR_STREAM("\n######\n");
  	surface_ptr->print();
  	ROS_ERROR_STREAM("\n######\n");
  	return surface_ptr;
}

void placement_module::read_object_models() {

	std::string resources_path = ros::package::getPath("chimp_resources");
	ROS_ERROR_STREAM("Executing Command");
	ROS_ERROR_STREAM("Executing Command");
	ROS_ERROR_STREAM("Executing Command");
	ROS_ERROR_STREAM("Executing Command");
	ROS_ERROR_STREAM("rosparam load " + resources_path + "/config/obj_config.yaml");
	ROS_ERROR_STREAM("Executing Command");
	ROS_ERROR_STREAM("Executing Command");
	ROS_ERROR_STREAM("Executing Command");
	ROS_ERROR_STREAM("Executing Command");
	system(("rosparam load " + resources_path + "/config/obj_config.yaml").c_str());

	ros::Duration(1).sleep();
	int num_objects = 0;
		
	if(nh.hasParam("/iiwa/objects/num_objects"))
		nh.getParam("/iiwa/objects/num_objects", num_objects);
	else
	{
		ROS_WARN_STREAM("Could not read the object config file...");
		exit(1);
	}

	for(int ii=0; ii<num_objects; ii++){
		char obj_topic[50];
		std::string obj_name;
		std::string model_location;

		sprintf(obj_topic, "/objects/object_%d", ii+1);
		nh.getParam((std::string(obj_topic) + "/name").c_str(), obj_name);
		nh.getParam((std::string(obj_topic) + "/model_location").c_str(), model_location);

		std::cout << "Loaded Object " << ii+1 << " : " << obj_name << std::endl;
		
		model_location = resources_path + "/" + model_location;
		ROS_ERROR_STREAM("OBJECT LOCATION: " << "Loaded Object " << ii+1 << " : " << obj_name << "" << model_location <<std::endl);

		pcl::PointCloud<pcl::PointNormal>::Ptr pcl_model = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
		pcl::io::loadPLYFile(model_location, *pcl_model);

		extract_surfaces_cuboid(obj_name, pcl_model);

		// int input = 0;
		// std::cout<<"Enter -1 to exit...";
		// std::cin>>input;
		// if(input<0)
		// {
		// 	break;
		// }
	}



	

	if(ros::param::has("/iiwa/objects/num_objects"))
	{
		int num_objs = 0;
		ros::param::get("/iiwa/objects/num_objects", num_objs);

		for(int i=0; i<num_objs; ++i)
		{
			std::string prefix = "/iiwa/objects/object_"+std::to_string(i+1);
			std::string obj_name;
			ros::param::get((std::string(prefix+"/name")).c_str(), obj_name);
			ROS_WARN_STREAM("Reading object: "<<obj_name);
			std::vector< Eigen::Vector3f > normals;

			if(ros::param::has(prefix+"/grasp_normals"))
			{
				std::vector<double> vec;
				// ros::param::get(prefix+"/grasp_normals", normals);
				nh.getParam(prefix+"/grasp_normals", vec);

				if(vec.size()%3 != 0)
				{
					ROS_ERROR_STREAM("Grasp normals list malformed.");
				}
				else
				{
					for(int read_index = 0; read_index<vec.size(); read_index+=3)
					{
						normals.push_back(Eigen::Vector3f({vec[read_index], vec[read_index+1], vec[read_index+2]}));
					}
				}
				ROS_WARN_STREAM("\n#############################################\n#############################################\n#############################################\n#############################################\n#############################################\nRead the grasp normals for the object "
					<<obj_name<<" [ "<<normals.size()<<
					" ]\n#############################################\n#############################################\n#############################################\n#############################################\n#############################################\n");
				for(auto normal: normals)
				{
					ROS_ERROR_STREAM("Normal:\n"<<normal);
				}
			}

			grasp_normals[obj_name] = normals;
			ROS_WARN_STREAM("Read grasp normals: "<<normals.size());

			for(auto surface: surface_map[obj_name])
			{
				bool normal_present = false;
				for(auto normal: grasp_normals[obj_name])
				{
					if(normal.dot(surface->surface_normal()) > 0.9)
					{
						normal_present = true;
						break;
					}
				}
				if(normal_present)
				{
					surface->graspable = true;
				}
				else
				{
					surface->graspable = false;
				}

				ROS_ERROR_STREAM("Parsing surfaces");
				surface->print();
			}

			// std::string input;
			// std::cin>>input;
		}



	}
	else
	{
		ROS_WARN_STREAM("Could not read the object config file...");
		exit(1);
	}



}