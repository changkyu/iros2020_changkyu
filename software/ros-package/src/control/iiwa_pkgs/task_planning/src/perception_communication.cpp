#include <perception_communication.hpp>

Eigen::Vector3f global_surface_center_world;
Eigen::Vector3f global_secondary_axis;	
ros::Publisher marker_pub;
ros::Publisher object_marker_pub;

std::map<std::string, std::string > mesh_map;
geometry_msgs::Pose latest_detected_object_pose;
std::string latest_detected_object_name;


perception_communication::perception_communication()
{

	set_estimation_pub = nh.advertise<chimp_pose_estimation::SetObjectNameBinIdMsg>("/set_sensing_info",1);
	get_estimation_client = nh.serviceClient<chimp_pose_estimation::GetObjectPose>("/get_pose_estimate");
 	marker_pub = nh.advertise<visualization_msgs::Marker>("Secondary_axis", 1);
 	object_marker_pub = nh.advertise<visualization_msgs::Marker>("Object_detection", 1);
 	latest_detected_object_name = "dove";
 	latest_detected_object_pose.position.x = 0;
 	latest_detected_object_pose.position.y = 0;
 	latest_detected_object_pose.position.z = 0;
 	latest_detected_object_pose.orientation.x = 0;
 	latest_detected_object_pose.orientation.y = 0;
 	latest_detected_object_pose.orientation.z = 0;
 	latest_detected_object_pose.orientation.w = 1;

	// #ifdef REFLEX
		read_object_models();
	// #endif

	surface_to_grasp_offset = 0.01;
	bin_id = 2;
}


void perception_communication::extract_surfaces_cuboid(std::string obj_name, pcl::PointCloud<pcl::PointNormal>::Ptr pcl_model) {

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

object_surface* perception_communication::get_surface(Eigen::Vector3f& position_OBB_vec, std::vector<Eigen::Vector3f>& bounds, 
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

  	return surface_ptr;
}

void perception_communication::read_object_models() {

	std::string resources_path = ros::package::getPath("chimp_resources");
	system(("rosparam load " + resources_path + "/config/obj_config.yaml").c_str());

	ros::Duration(1).sleep();
	int num_objects = 0;
	// if(nh.hasParam("/iiwa/objects/num_objects"))
	// 	ROS_WARN_STREAM("Parameter with full path exists");
	// else
	// 	ROS_WARN_STREAM("Parameter with full path DOES NOT exist");
		
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
		auto temp = model_location;

		mesh_map[obj_name] = temp.substr(0,temp.find_last_of("/\\")) + "/" + obj_name + ".obj";
		mesh_map[obj_name] = "package://chimp_resources/models_search/"+obj_name+"/"+obj_name+".obj";
		mesh_map[obj_name] = "package://chimp_resources/models_search/"+obj_name+"/model_search.ply";


		#ifdef REFLEX
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
		#endif
	}
	
}

std::vector<response_t> perception_communication::get_sensing_from_object_pose(std::string label, geometry_msgs::Pose pose)
{

	std::string highest_object;
	config_t highest_ee_pose(0,0,-100000,0,0,0,0);
	double highest_surface_width;
	object_surface* highest_surface;

	std::vector<response_t> response_vec;
	ROS_WARN_STREAM("Object: "<<label<<" at "<<pose);

	auto surfaces = surface_map[label];

	Eigen::Vector3f reference_direction(0,0,1);
	double max_dot_product = -10000;
	object_surface* top_facing_surface = NULL;

	for(auto surface: surfaces)
	{
		double dot_product = get_dot_product_of_surface_normal(pose, surface, reference_direction);
		if(dot_product>max_dot_product)
		{
			ROS_WARN_STREAM("For object: "<<label<<" more promising surface dot product "<<dot_product << " / "<<max_dot_product);
			top_facing_surface = surface;
			max_dot_product = dot_product;
		}
	}

	auto ee_pose = compute_ee_pose(pose, top_facing_surface);
	if(ee_pose.z > highest_ee_pose.z)
	{
		ROS_WARN_STREAM("For object: "<<label<<" more promising surface z "<< ee_pose.print() << " / "<< ee_pose.print());
		
		highest_ee_pose = ee_pose;
		highest_ee_pose.z-=surface_to_grasp_offset;
		highest_object = label;
		// highest_surface_width = top_facing_surface->dimensions[1];
		highest_surface_width = get_projected_width(pose, top_facing_surface);
		highest_surface = top_facing_surface;
	}

	response_t response(highest_object, highest_ee_pose, highest_surface_width);
	response_vec.push_back(response);
	std::cout<<"\nResponse... "<<response.print()<<"\n";
	highest_surface->print();

	return response_vec;

}

std::vector<response_t> perception_communication::get_sensing()
{
	ROS_WARN_STREAM("Invoking blocking service call to fetch sensed objects.");
	std::string highest_object;
	config_t highest_ee_pose(0,0,-100000,0,0,0,0);
	config_t highest_detection(0,0,-100000,0,0,0,0);
	double highest_surface_width;
	object_surface* highest_surface;

	std::vector<response_t> response_vec;

	if (get_estimation_client.call(get_estimation_srv))
	{
		if(!get_estimation_srv.response.Objects.empty())
		{

#ifdef REFLEX

				for(auto object : get_estimation_srv.response.Objects)
				{

					#ifdef USE_GAZEBO
					
						spawnObject(nh, "/home/cm1074/.gazebo/models/"+object.label+"/model-1_4.sdf", 
							object.label, object.pose);
					#endif
					auto label = object.label;
					auto pose = object.pose;

					ROS_WARN_STREAM("Object: "<<label<<" at "<<pose);

					auto surfaces = surface_map[label];

					Eigen::Vector3f reference_direction(0,0,1);
					double max_dot_product = -10000;
					object_surface* top_facing_surface = NULL;

					for(auto surface: surfaces)
					{
						double dot_product = get_dot_product_of_surface_normal(pose, surface, reference_direction);
						if(dot_product>max_dot_product)
						{
							ROS_WARN_STREAM("For object: "<<label<<" more promising surface dot product "<<dot_product << " / "<<max_dot_product);
							top_facing_surface = surface;
							max_dot_product = dot_product;
						}
					}

					auto ee_pose = compute_ee_pose(pose, top_facing_surface);
					if(ee_pose.z > highest_ee_pose.z)
					{
						ROS_WARN_STREAM("For object: "<<label<<" more promising surface z "<< ee_pose.print() << " / "<< ee_pose.print());
						
						highest_ee_pose = ee_pose;
						highest_ee_pose.z-=surface_to_grasp_offset;
						highest_object = label;
						// highest_surface_width = top_facing_surface->dimensions[1];
						highest_surface_width = get_projected_width(pose, top_facing_surface);
						highest_surface = top_facing_surface;
					}
				}

#endif
			
#ifndef REFLEX
			if(get_estimation_srv.response.Detections.size()>0)
			{
				auto detection = get_estimation_srv.response.Detections[0];
				highest_detection = compute_ee_pose(detection.pose);
			}

			auto object = get_estimation_srv.response.Objects[0];
			#ifdef USE_GAZEBO				
				spawnObject(nh, "/home/cm1074/.gazebo/models/"+object.label+"/model-1_4.sdf", object.label, object.pose);
			#endif
			highest_object = object.label;
			highest_surface_width = 0.04;
			highest_ee_pose = compute_ee_pose(object.pose);
			
#endif
			// response_t response(highest_object, highest_ee_pose, highest_surface_width);
			response_t response(highest_object, highest_ee_pose, highest_detection, highest_surface_width, get_estimation_srv.response.Regrasp);
			response_vec.push_back(response);
			std::cout<<"\nResponse... "<<response.print()<<"\n";
			// highest_surface->print();
		}
	}
	else 
	{
		ROS_WARN_STREAM("Failed to get objects from estimation service...");
		return response_vec;
	}

	latest_detected_object_pose = highest_detection.get_pose();
	latest_detected_object_name = highest_object;

	return response_vec;
	
}

void perception_communication::start_sensing()
{
	/*
		int16 binId
		string[] ObjectNames
		---
		ObjectPose[] Objects
	
		ObjectPose
		string label
		geometry_msgs/Pose pose
	*/
	chimp_pose_estimation::SetObjectNameBinIdMsg msg;
	msg.binId = bin_id;
	msg.ObjectNames = object_list;
	msg.BinDimensions = bounding_box_dimensions;
	msg.BinPose7D = bounding_box_pose;

	ROS_INFO_STREAM("calling sense on bin "<<bin_id);
	set_estimation_pub.publish(msg);

}

double perception_communication::get_dot_product_of_surface_normal(geometry_msgs::Pose object_pose, object_surface* surface, Eigen::Vector3f reference_direction)
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
	dot_product = surface_normal_world_frame.dot(reference_direction);
	
	//Normalize surface normal
	//Normalize reference direction
	//Calculate the dot product

	return dot_product;
}

double perception_communication::get_projected_width(geometry_msgs::Pose object_pose, object_surface* surface)
{
	double _width;
	//Transform surface->normal to world frame
	Eigen::Quaternionf q;
	q.w() = object_pose.orientation.w;
	q.x() = object_pose.orientation.x;
	q.y() = object_pose.orientation.y;
	q.z() = object_pose.orientation.z;
	Eigen::Matrix3f rotMat;
	rotMat = q.toRotationMatrix();

	Eigen::Vector3f secondary_axis = surface->dimensions[1] * rotMat.col(1);

	secondary_axis[2] = 0;
	_width = secondary_axis.norm();

	return _width;
}

void check_rotation_matrix(Eigen::Matrix3f &input_rotation)
{
	std::cout<<"################CHECK################\n";
	std::cout<< "Input rotation matrix: \n"<<input_rotation<<"\n";
	Eigen::Quaternionf _quaternion(input_rotation);
	// std::cout << "Converted Quaternion: \n"<<_quaternion<<"\n";
	input_rotation = _quaternion.toRotationMatrix();
	std::cout<< "Reverted rotation matrix: \n"<< input_rotation <<"\n";
	std::cout<<"###############CHECKED###############\n";

}

void publish_arrow(const ros::TimerEvent& event)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Point p1, p2;
  float arrow_size = 0.1;
  p1.x = global_surface_center_world[0];
  p1.y = global_surface_center_world[1];
  p1.z = global_surface_center_world[2];

  p2.x = global_surface_center_world[0];
  p2.y = global_surface_center_world[1];
  p2.z = global_surface_center_world[2] + arrow_size;


	// P2.x = x*(qx*qx+qw*qw-qy*qy- qz*qz) + y*(2*qx*qy- 2*qw*qz) + z*(2*qx*qz+ 2*qw*qy)
	// P2.y = x*(2*qw*qz + 2*qx*qy) + y*(qw*qw - qx*qx+ qy*qy - qz*qz)+ z*(-2*qw*qx+ 2*qy*qz)
	// P2.z = x*(-2*qw*qy+ 2*qx*qz) + y*(2*qw*qx+ 2*qy*qz)+ z*(qw*qw - qx*qx- qy*qy+ qz*qz)



  marker.points.push_back(p1);
  marker.points.push_back(p2);

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  // std::cout << p1.x << " " << p1.y << " " << p1.z << " " << p2.x << " " << p2.y << " " << p2.z << std::endl;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.01;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;

  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);



  	visualization_msgs::Marker object_marker;
	object_marker.header.frame_id = "/world";
	object_marker.header.stamp = ros::Time::now();
	object_marker.ns = "object_detection";
	object_marker.id = 0;
	// object_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	object_marker.type = visualization_msgs::Marker::CUBE;
	object_marker.action = visualization_msgs::Marker::ADD;
	object_marker.pose = latest_detected_object_pose;
	// object_marker.scale.x = 0.12;
	// object_marker.scale.y = 0.04;
	// object_marker.scale.z = 0.08;
	object_marker.scale.x = 0.095;
	object_marker.scale.y = 0.036;
	object_marker.scale.z = 0.068;
	// object_marker.scale.x = 0.234;
 //    object_marker.scale.y = 0.050;
 //    object_marker.scale.z = 0.041;
	object_marker.color.a = 1.0; // Don't forget to set the alpha!
	object_marker.color.r = 1.0;
	object_marker.color.g = 0.0;
	object_marker.color.b = 0.0;
	//only if using a MESH_RESOURCE object_marker type:
	// object_marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  	object_marker.lifetime = ros::Duration();
	object_marker.mesh_resource = mesh_map[latest_detected_object_name];
	// ROS_ERROR_STREAM("Visualizing.. "<<latest_detected_object_name<<" with "<<mesh_map[latest_detected_object_name]);
	object_marker_pub.publish( object_marker );
}

config_t perception_communication::compute_ee_pose(geometry_msgs::Pose object_pose, object_surface* surface)
{
#ifndef REFLEX
		ROS_ERROR_STREAM("Compute EE Pose for the reflex should not be invoked. ");
		exit(1);
		return config_t();
#endif

#ifdef REFLEX
	Eigen::Quaternionf q;
	q.w() = object_pose.orientation.w;
	q.x() = object_pose.orientation.x;
	q.y() = object_pose.orientation.y;
	q.z() = object_pose.orientation.z;
	Eigen::Matrix3f object_rotation = q.toRotationMatrix();
	Eigen::Vector3f object_translation(object_pose.position.x, 
								object_pose.position.y,
								object_pose.position.z);

	check_rotation_matrix(surface->rotation);

	Eigen::Matrix3f surface_world_frame = object_rotation*surface->rotation;

	// project the secondary axis of the object to x-y plane
	surface_world_frame(2,1) = 0;
	surface_world_frame.col(1) = surface_world_frame.col(1).normalized();

	// the tertiary axis is aligned to negative z of world frame
	surface_world_frame.col(2) = Eigen::Vector3f(0,0,-1);

	// primary axis is computed by cross product of the 2
	surface_world_frame.col(0) = surface_world_frame.col(1).cross(surface_world_frame.col(2));
	check_rotation_matrix(surface_world_frame);

	// rotate the ee rotation by 45 degrees about local z of end effector
	Eigen::Quaternionf q_ee_offset;
	q_ee_offset.w() = 0.924;
	q_ee_offset.x() = 0;
	q_ee_offset.y() = 0;
	q_ee_offset.z() = 0.383;
	Eigen::Matrix3f rotation_ee_offset = q_ee_offset.toRotationMatrix();

	Eigen::Matrix3f final_ee_rotation = rotation_ee_offset*surface_world_frame;

	std::cout << "object rotation: " << object_rotation << std::endl;
	std::cout << "surface->center: " << surface->center << std::endl;
	std::cout << "object_translation: " << object_translation << std::endl;
	
	Eigen::Vector3f surface_center_world = object_rotation*surface->center + object_translation;

	//########################## Z OFFSET ########################
	Eigen::Vector3f secondary_axis_measure = surface_world_frame.col(1);
	// publish_arrow(surface_center_world, secondary_axis_measure);
	global_surface_center_world = surface_center_world;
	global_secondary_axis = secondary_axis_measure;

	secondary_axis_measure = secondary_axis_measure*( (surface->dimensions[1]*0.5) + 0.005);
	double z_offset = -(std::abs(secondary_axis_measure[2]));
	//#### ROBOTIQ
	// surface_center_world[2] += z_offset;
	//########################## Z OFFSET ########################
	

	std::cout << "surface_center_world: " << surface_center_world << std::endl;
	config_t return_config(surface_center_world, final_ee_rotation);

	// #### ROBOTIQ
	if (bin_id==1)
	{
		return_config.qx = 0.34554;
		return_config.qy = 0.93840;
		return_config.qz = -0.001849;
		return_config.qw = -0.000465;
	}
	else if(bin_id==2)
	{
		// return_config.qx = -0.170039;
		// return_config.qy = 0.98463;
		// return_config.qz = -0.0095502;
		// return_config.qw = -0.0387;

		return_config.qx = -0.334696;
		return_config.qy = 0.94232;
		return_config.qz = -0.00172;
		return_config.qw = 0.00083;


	}
	else
	{
		return_config.qx = 1;
		return_config.qy = 0;
		return_config.qz = 0;
		return_config.qw = 0;
	}

	return return_config;
	//Transform surface->normal to world frame
	//Align the ee with secondary axis *account for 45' offset
#endif
}


config_t perception_communication::compute_ee_pose(geometry_msgs::Pose object_pose)
{
	Eigen::Vector3f surface_center_world(object_pose.position.x, 
								object_pose.position.y,
								object_pose.position.z);
	Eigen::Quaternionf q;
	q.w() = object_pose.orientation.w;
	q.x() = object_pose.orientation.x;
	q.y() = object_pose.orientation.y;
	q.z() = object_pose.orientation.z;
	// Eigen::Matrix3f final_ee_rotation;
	// final_ee_rotation.setIdentity();
	Eigen::Matrix3f final_ee_rotation = q.toRotationMatrix();

	global_surface_center_world = surface_center_world;
	
	config_t return_config(surface_center_world, final_ee_rotation);

	return return_config;
}


void spawnObject(ros::NodeHandle &node_handle_, 
    const std::string &file_name, 
    const std::string &model_name, 
    geometry_msgs::Pose pose)
{

	ros::ServiceClient gazebo_spawn_clt_ 
		= node_handle_.serviceClient< gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");

	gazebo_msgs::SpawnModel model;
	std::ifstream ifs(file_name.c_str());

	std::string line;
	while(!ifs.eof()) // Parse the contents of the given urdf in a string
	{
		std::getline(ifs,line);
		model.request.model_xml+=line;
	}
	ifs.close();

	model.request.model_name = model_name;
	model.request.reference_frame = "world";
	model.request.initial_pose = pose;

	gazebo_spawn_clt_.call(model); 

}