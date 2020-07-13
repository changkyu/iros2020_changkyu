/*
File: MoveItNode.cpp

Authors: Aravind Sivaramakrishnan

Description: A subscriber that listens to a topic that publishes the desired Cartesian Pose, and uses MoveIt!
to plan and execute to go to that particular pose.

Comments/TODO:
- Sometimes, execution completes with state "ABORTED", though this doesn't affect future planning.
- These dim and pose of the table, wall etc. are not very reliable at this stage.
- Too messy to edit the object attributes and build every time. Should probably do this through a .launch file.
- Also, should get around to adding the gripper, etc. to the urdf...
- Have to deal with different shape primitives, though this is a not a huge priority right now.
- Adding the box only for the JD X demo. Add it through the task planner call later.
*/
#include <MoveItNode.hpp>
//  #define _DEBUG
ros::Publisher marker_pub;
ros::Publisher cube_pub;
int marker_id = 0;
//#define CAMERA_CALIBRATION
void publish_cylinder(geometry_msgs::Pose pose)
{

  // char markerId[30];
// sprintf(markerId, "Object%04d", rand()%9999);
  std::cout<<"Publishing marker..."<<std::endl;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = marker_id;
  marker_id ++;
  // Set the marker type
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  // geometry_msgs::Point p1, p2;
  // float arrow_size = 0.1;
  // p1.x = global_surface_center_world[0];
  // p1.y = global_surface_center_world[1];
  // p1.z = global_surface_center_world[2];
  // p2.x = global_surface_center_world[0] + arrow_size*global_secondary_axis[0];
  // p2.y = global_surface_center_world[1] + arrow_size*global_secondary_axis[1];
  // p2.z = global_surface_center_world[2] + arrow_size*global_secondary_axis[2];
  // marker.points.push_back(p1);
  // marker.points.push_back(p2);
  pose.position.z -= 0.6;
  marker.pose = pose;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.7;

  // std::cout << p1.x << " " << p1.y << " " << p1.z << " " << p2.x << " " << p2.y << " " << p2.z << std::endl;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.15;

  marker.lifetime = ros::Duration(10000);
  marker_pub.publish(marker);
  ros::spinOnce();


  // visualization_msgs::Marker cube;
  // // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  // cube.header.frame_id = "/world";
  // cube.header.stamp = ros::Time::now();

  // // Set the namespace and id for this cube.  This serves to create a unique ID
  // // Any cube sent with the same namespace and id will overwrite the old one
  // cube.ns = "basic_cube";
  // cube.id = 0;

  // // Set the cube type
  // cube.type = visualization_msgs::Marker::CUBE;
  // cube.action = visualization_msgs::Marker::ADD;

  // // geometry_msgs::Point p1, p2;
  // // float arrow_size = 0.1;
  // // p1.x = global_surface_center_world[0];
  // // p1.y = global_surface_center_world[1];
  // // p1.z = global_surface_center_world[2];
  // // p2.x = global_surface_center_world[0] + arrow_size*global_secondary_axis[0];
  // // p2.y = global_surface_center_world[1] + arrow_size*global_secondary_axis[1];
  // // p2.z = global_surface_center_world[2] + arrow_size*global_secondary_axis[2];
  // // cube.points.push_back(p1);
  // // cube.points.push_back(p2);

  // cube.pose = pose;
  // cube.color.r = 0.0f;
  // cube.color.g = 1.0f;
  // cube.color.b = 0.0f;
  // cube.color.a = 0.7;

  // // std::cout << p1.x << " " << p1.y << " " << p1.z << " " << p2.x << " " << p2.y << " " << p2.z << std::endl;
  // // Set the scale of the cube -- 1x1x1 here means 1m on a side
  // cube.scale.x = 0.05;
  // cube.scale.y = 0.05;
  // cube.scale.z = 0.05;

  // cube.lifetime = ros::Duration(10000);
  // cube_pub.publish(cube);

  
  ros::spinOnce();
}


void MoveItNode::get_bin_poses(std::vector<float>& bin_pose_1, std::vector<float>& bin_pose_2)
{
	//std::string resources_path = ros::package::getPath("chimp_resources");
    std::string resources_path = ros::package::getPath("pkg_ijcai2019");
	system(("rosparam load " +resources_path + "/config/bin_config.yaml").c_str());
// read bin pose

	node_handle.getParam("bins/bin_1/bin_pose", bin_pose_1);
	node_handle.getParam("bins/bin_2/bin_pose", bin_pose_2);
	convert_wxyz_to_xyzw(bin_pose_1);
	convert_wxyz_to_xyzw(bin_pose_2);

	ROS_WARN_STREAM("Loaded Bin pose 1: "<<bin_pose_1[0]<<","<<bin_pose_1[1]<<","<<bin_pose_1[2]<<","<<bin_pose_1[3]<<","<<bin_pose_1[4]<<","<<bin_pose_1[5]<<","<<bin_pose_1[6]<<","<<bin_pose_1[7]);
	ROS_WARN_STREAM("Loaded Bin pose 2: "<<bin_pose_2[0]<<","<<bin_pose_2[1]<<","<<bin_pose_2[2]<<","<<bin_pose_2[3]<<","<<bin_pose_2[4]<<","<<bin_pose_2[5]<<","<<bin_pose_2[6]<<","<<bin_pose_2[7]);
	// int x;
	// std::cin>>x;
}

void MoveItNode::convert_wxyz_to_xyzw(std::vector<float>& pose)
{
	// 0.6123724, 0.6123724, 0.3535534, 0.3535534
	// 0.6123724,0.3535534,0.3535534,0.6123724
	float temp_w = pose[3];
	pose[3]=pose[4];
	pose[4]=pose[5];
	pose[5]=pose[6];
	pose[6]=temp_w;
}

void MoveItNode::deleteBin(){
	moveit_msgs::CollisionObject bin_object;
	bin_object.id = "big_bin";
	bin_object.operation = bin_object.REMOVE;
	collision_objects.push_back(bin_object);
	plan.applyCollisionObjects(collision_objects);
}

void MoveItNode::addBin(){
	moveit_msgs::CollisionObject bin_object;
	std::vector<float> bin_pose1;
	std::vector<float> bin_pose2;
	std::vector<float> bin_pad_pose;
	get_bin_poses(bin_pose1, bin_pose2);
	// std::vector<float> boxDim = {0.8,0.4,0.25};
	// float boxOffset = -0.18; //	-(0.2+boxDim.at(2)/2);
	// std::vector<float> boxPose = {0.65,-0.45,boxOffset,1,0,0,0};
	// addCollisionObject(box_object,"box",boxDim,boxPose);

	addCollisionObjectFromMesh(bin_object, "package://motion_planning/models/accurate_box.obj", bin_pose1, "big_bin");
	collision_objects.push_back(bin_object);
	// std::vector<float> right_cameraDim = {0.24, 0.85, 0.9};// x:0.65 {0.15, 0.5, 0.9}
	// moveit_msgs::CollisionObject right_camera_object;
	// //std::vector<float> right_cameraPose = {0.71, -0.46, 0.20, 0, 0, -0.2419219, 0.9702957}; // 0.71
	// std::vector<float> right_cameraPose = {0.685, -0.503, 0.20, 0, 0, -0.2419219, 0.9702957}; // 0.71
	
	// addCollisionObject(right_camera_object, "camera_R", right_cameraDim, right_cameraPose);
	// collision_objects.push_back(right_camera_object);
	plan.applyCollisionObjects(collision_objects);
}

MoveItNode::MoveItNode():group("manipulator"){
	planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	while(planning_scene_diff_publisher.getNumSubscribers() < 1)
	{
	  ros::Duration(5.0).sleep();
	}

	// Setting some planning params.
	group.setPlanningTime(10.0);
	group.allowReplanning(true);
	group.setEndEffectorLink("iiwa_link_ee");
	group.setPlannerId("RRTConnectkConfigDefault");
	group.setMaxVelocityScalingFactor(1);
marker_pub = node_handle.advertise<visualization_msgs::Marker>("Collision_checking_region", 1);
  cube_pub = node_handle.advertise<visualization_msgs::Marker>("success_region", 1);
	//addGripperObject();
	my_iiwa.init();

	
	// Adding the different collision objects.
	moveit_msgs::CollisionObject table_object, ceiling_object, wall_object, fake_object, bin_pad_object;

	//std::vector<float> tableDim = {1.2,1.8,0.2};
    //std::vector<float> tableDim = {0.80,1.54,0.2};
    std::vector<float> tableDim = {1.00,0.50,1.00};
	float tableOffset = -(0.29+tableDim.at(2)/2)+0.0325 + 0.04;
	//std::vector<float> tablePose = {0.4,0.0,tableOffset,1,0,0,0};
    //std::vector<float> tablePose = {0.53,0.0,tableOffset,1,0,0,0};
    //std::vector<float> tablePose = {0.53,0.0,-tableDim.at(2)*0.5,1,0,0,0};
    std::vector<float> tablePose = {0.13+tableDim.at(1),0.0,-tableDim.at(2)*0.5,1,0,0,0};

	std::vector<float> ceilingDim = {2.0,2.0,0.1};
	// float ceilingOffset = (1.35-ceilingDim.at(2)/2)-0.05;
	float ceilingOffset = (1.50-ceilingDim.at(2)/2)-0.25;
	std::vector<float> ceilingPose = {0.5,0.0,ceilingOffset+0.3,0,0,0,1};

	std::vector<float> wallDim = {0.2,1.0,2.0};
	// float wallOffset = -(0.85+wallDim.at(0)/2);
	float wallOffset = -(0.5+wallDim.at(0)/2);
	std::vector<float> wallPose = {wallOffset,0,0.25,0,0,0,1};

	std::vector<float> fakeObjDim = {0.15,0.15,0.15};
	// float wallOffset = -(0.85+wallDim.at(0)/2);
	std::vector<float> fakeObjPose = {0.5,0,0.13,0,0,0,1};

	addCollisionObject(table_object, "table_round", tableDim, tablePose);
	//addCollisionObject(ceiling_object, "ceiling", ceilingDim, ceilingPose);
	addCollisionObject(wall_object, "wall", wallDim, wallPose);
	addCollisionObject(fake_object, "fake", fakeObjDim, fakeObjPose);
	collision_objects.push_back(table_object);
	//#ifndef USE_GAZEBO
	//collision_objects.push_back(ceiling_object);
	//#endif
	collision_objects.push_back(wall_object);
	//collision_objects.push_back(fake_object);

#if 0
	moveit_msgs::CollisionObject bin_object;
	float bin_offset = -0.145; //	-(0.2+boxDim.at(2)/2);  //0.7071068,0,0,0.7071068
	// float bin_offset = -0.075; //	-(0.2+boxDim.at(2)/2);  //0.7071068,0,0,0.7071068
	// std::vector<double> bin_pose1 = {0.39,-0.34,bin_offset,0.6123724,0.3535534,0.3535534,0.6123724};
	// std::vector<double> bin_pose2 = {0.39,0.34,bin_offset,0.6123724,-0.3535534,-0.3535534,0.6123724};

	std::vector<float> bin_pose1;
	std::vector<float> bin_pose2;
	std::vector<float> bin_pad_pose;
	//get_bin_poses(bin_pose1, bin_pose2);
	// std::vector<float> boxDim = {0.8,0.4,0.25};
	// float boxOffset = -0.18; //	-(0.2+boxDim.at(2)/2);
	// std::vector<float> boxPose = {0.65,-0.45,boxOffset,1,0,0,0};
	// addCollisionObject(box_object,"box",boxDim,boxPose);
	// addCollisionObjectFromMesh(bin_object, "package://motion_planning/models/accurate_box.obj", bin_pose1, "big_bin");
	// collision_objects.push_back(bin_object);

#if 0 
    //[changkyu]    
	addCollisionObjectFromMesh(bin_object, "package://motion_planning/models/newest_bin.obj", bin_pose2, "small_bin");
	collision_objects.push_back(bin_object);
#endif	

	moveit_msgs::CollisionObject left_camera_object;
	#ifndef CAMERA_CALIBRATION
	// std::vector<float> left_cameraDim = {0.28, 0.5, 0.9};// {0.15, 0.5, 0.9}
	std::vector<float> left_cameraDim = {0.27, 0.85, 0.9};// {0.15, 0.5, 0.9}
	#else
	std::vector<float> left_cameraDim = {0.19, 0.85, 0.9};// {0.15, 0.5, 0.9}
	#endif
	//std::vector<float> left_cameraPose = {0.71, 0.46, 0.20,0, 0, 0.2419219, 0.9702957};
	std::vector<float> left_cameraPose = {0.685, 0.503, 0.20,0, 0, 0.2419219, 0.9702957};
	std::vector<float> bin_padPose = {0.495, 0.463, -0.14,0, 0, 0.258819, 0.9659258};

	std::vector<float> bin_padDim = {0.06, 0.7, 0.3};

	/* [changkyu] comment out
	addCollisionObject(left_camera_object, "camera_L", left_cameraDim, left_cameraPose);
	addCollisionObject(bin_pad_object, "bin_pad", bin_padDim, bin_padPose);
	*/

	moveit_msgs::CollisionObject right_camera_object, rod_object;
	#ifndef CAMERA_CALIBRATION
	// std::vector<float> right_cameraDim = {0.28, 0.5, 0.9};// x:0.65 {0.15, 0.5, 0.9}
	std::vector<float> right_cameraDim = {0.20, 0.14, 0.15};// x:0.65 {0.15, 0.5, 0.9}
	#else
	std::vector<float> right_cameraDim = {0.20, 0.20, 0.15};// x:0.65 {0.15, 0.5, 0.9}
	#endif
	//std::vector<float> right_cameraPose = {0.71, -0.46, 0.20, 0, 0, -0.2419219, 0.9702957}; // 0.71
	std::vector<float> right_cameraPose = {0.685, -0.503, 0.330, 0, 0, -0.2419219, 0.9702957}; // 0.71
	std::vector<float> rodPose = {0.685, -0.503, 0.20, 0, 0, -0.2419219, 0.9702957}; // 0.71
	std::vector<float> rodDim = {0.14, 0.65, 0.9};// x:0.65 {0.15, 0.5, 0.9}
	
	/* [changkyu] comment out
	addCollisionObject(rod_object, "rod_R", rodDim, rodPose);
	addCollisionObject(right_camera_object, "camera_R", right_cameraDim, right_cameraPose);
	*/
	
	collision_objects.push_back(left_camera_object);
	collision_objects.push_back(right_camera_object);
    
    // [changkyu]
    //collision_objects.push_back(bin_pad_object);
    
	collision_objects.push_back(rod_object);
#endif

	plan.applyCollisionObjects(collision_objects);

#if 0

	transfer_pose_1.position.x = 0.39;
	transfer_pose_1.position.y = -0.36;
	transfer_pose_1.position.z = 0.55;
	transfer_pose_1.orientation.x = -0.38268343;
	transfer_pose_1.orientation.y = 0.92387953;
	transfer_pose_1.orientation.z = 0.0;
	transfer_pose_1.orientation.w = 0.0;

	transfer_pose_2.position.x = 0.39;
	transfer_pose_2.position.y = 0.34;
	transfer_pose_2.position.z = 0.55;
	transfer_pose_2.orientation.x = -0.38268343;
	transfer_pose_2.orientation.y = 0.92387953;
	transfer_pose_2.orientation.z = 0.0;
	transfer_pose_2.orientation.w = 0.0;

	bin_1_go_pose.position.x = 0.39;
	bin_1_go_pose.position.y = -0.36;
	bin_1_go_pose.position.z = 0.55;
	bin_1_go_pose.orientation.x = -0.38268343;
	bin_1_go_pose.orientation.y = 0.92387953;
	bin_1_go_pose.orientation.z = 0.0;
	bin_1_go_pose.orientation.w = 0.0;
	
	bin_1_back_pose.position.x = 0.39;
	bin_1_back_pose.position.y = 0.34;
	bin_1_back_pose.position.z = 0.4;
	bin_1_back_pose.orientation.x = -0.38268343;
	bin_1_back_pose.orientation.y = 0.92387953;
	bin_1_back_pose.orientation.z = 0.0;
	bin_1_back_pose.orientation.w = 0.0;


	
	bin_2_go_pose.position.x = 0.39;
	bin_2_go_pose.position.y = 0.34;
	bin_2_go_pose.position.z = 0.55;
	bin_2_go_pose.orientation.x = -0.38268343;
	bin_2_go_pose.orientation.y = 0.92387953;
	bin_2_go_pose.orientation.z = 0.0;
	bin_2_go_pose.orientation.w = 0.0;


	bin_2_back_pose.position.x = 0.39;
	bin_2_back_pose.position.y = -0.36;
	bin_2_back_pose.position.z = 0.4;
	bin_2_back_pose.orientation.x = -0.38268343;
	bin_2_back_pose.orientation.y = 0.92387953;
	bin_2_back_pose.orientation.z = 0.0;
	bin_2_back_pose.orientation.w = 0.0;
		
	bin_1_go_joint = { -0.694064617157
  ,0.328233838081
  ,-0.0120594855398
  ,-1.57092058659
  ,5.47439958609e-05
  ,1.24201476574
  ,0.000170116036315};
	// bin_1_go_joint = { -2.4064617157
 //  ,0.328233838081
 //  ,-0.0120594855398
 //  ,-1.57092058659
 //  ,5.47439958609e-05
 //  ,1.24201476574
 //  ,0.000170116036315};
	// bin_2_back_joint = { -0.694061458111
 //  ,0.513390719891
 //  , -0.0120597854257
 //  , -1.7465223074
 //  , 5.3210027545e-05
 //  , 0.914700210094
 //  ,0.000170295810676};
  bin_2_back_joint = { -0.694064617157
  ,0.328233838081
  ,-0.0120594855398
  ,-1.57092058659
  ,5.47439958609e-05
  ,1.24201476574
  ,0.000170116036315};
	bin_2_go_joint = {0.694064617157
  , 0.328233838081
  , -0.0120594855398
  , -1.57092058659
  , 5.47439958609e-05
  , 1.24201476574
  , 0.000170116036315};
  // bin_2_go_joint = {2.4064617157
  // , 0.328233838081
  // , -0.0120594855398
  // , -1.57092058659
  // , 5.47439958609e-05
  // , 1.24201476574
  // , 0.000170116036315};
	// bin_1_back_joint = { 0.694061458111
 //  , 0.513390719891
 //  , -0.0120597854257
 //  , -1.7465223074
 //  , 5.3210027545e-05
 //  , 0.914700210094
 //  , 0.000170295810676};
	bin_1_back_joint = {0.694064617157
  , 0.328233838081
  , -0.0120594855398
  , -1.5pre_computed7092058659
  , 5.47439958609e-05
  , 1.24201476574
  , 0.000170116036315};

	//pre_computed = true;
    pre_computed = false;

	if(pre_computed){
		int bin_total_id = 12;
		std::string saved_directory = "";
		
		std::string grid_path_filename_1to2, grid_path_filename_2to1;
		moveit::planning_interface::MoveGroup::Plan grid_plan;
		for(int id_bin_1 = 0; id_bin_1 < bin_total_id; id_bin_1++){
			for(int id_bin_2 = 0; id_bin_2 < bin_total_id; id_bin_2++){
				rosbag::Bag grid_bag_1to2, grid_bag_2to1;
				grid_path_filename_1to2 = saved_directory + "path_1_to_2_" + std::to_string(id_bin_1) + "_" + std::to_string(id_bin_2) +".bag";
				grid_path_filename_2to1 = saved_directory + "path_2_to_1_" + std::to_string(id_bin_1) + "_" + std::to_string(id_bin_2) +".bag";
				grid_bag_1to2.open(grid_path_filename_1to2, rosbag::bagmode::Read);
				//ROS_WARN_STREAM("open " <<grid_path_filename_1to2);

				grid_bag_2to1.open(grid_path_filename_2to1, rosbag::bagmode::Read);
				//ROS_WARN_STREAM("open " <<grid_path_filename_2to1);

				std::vector<std::string> topics;
				topics.push_back("/start_state");
				topics.push_back("/trajectory");
				topics.push_back("/planning_time");

				rosbag::View grid_view_1to2(grid_bag_1to2, rosbag::TopicQuery(topics));
				//ROS_WARN_STREAM("grid_view_1to2");
				rosbag::View grid_view_2to1(grid_bag_2to1, rosbag::TopicQuery(topics));
				//ROS_WARN_STREAM("grid_view_2to1");
				
				foreach(rosbag::MessageInstance const m, grid_view_1to2)
	    		{
	    			
	        		moveit_msgs::RobotState::Ptr s = m.instantiate<moveit_msgs::RobotState>();
	        		if (s != NULL){ 
	        			grid_plan.start_state_ = *s;
	        			ROS_WARN_STREAM(grid_plan.start_state_.joint_state.position[0]<<","<<grid_plan.start_state_.joint_state.position[1]);
	        		}
	        			moveit_msgs::RobotTrajectory::ConstPtr q = m.instantiate<moveit_msgs::RobotTrajectory>();
	        		
	        		if (q != NULL){
	            		grid_plan.trajectory_ = *q;
	            	}
	            		std_msgs::Float32::ConstPtr t = m.instantiate<std_msgs::Float32>();
	        		
	        		if (t != NULL)
	        			grid_plan.planning_time_ = t->data;

	        		
	    		}
	    		std::tuple<int, int, int, int> grid_tuple = std::make_tuple(1,2, id_bin_1, id_bin_2);
	    		ROS_WARN_STREAM("grid_plan has "<<grid_plan.trajectory_.joint_trajectory.points.size()<<" points");
	    		grid_saved_paths[grid_tuple] = grid_plan;

				foreach(rosbag::MessageInstance const m, grid_view_2to1)
	    		{
	    			
	        		moveit_msgs::RobotState::Ptr s = m.instantiate<moveit_msgs::RobotState>();
	        		if (s != NULL){ 
	        			grid_plan.start_state_ = *s;
	        			ROS_WARN_STREAM(grid_plan.start_state_.joint_state.position[0]<<","<<grid_plan.start_state_.joint_state.position[1]);
	        		}
	        			moveit_msgs::RobotTrajectory::ConstPtr q = m.instantiate<moveit_msgs::RobotTrajectory>();
	        		
	        		if (q != NULL){
	            		grid_plan.trajectory_ = *q;
	            	}
	            		std_msgs::Float32::ConstPtr t = m.instantiate<std_msgs::Float32>();
	        		
	        		if (t != NULL)
	        			grid_plan.planning_time_ = t->data;

	        		
	    		}

	    		grid_tuple = std::make_tuple(2,1, id_bin_1, id_bin_2);
	    		//ROS_WARN_STREAM("grid_plan has "<<grid_plan.trajectory_.joint_trajectory.points.size()<<" points");
	    		grid_saved_paths[grid_tuple] = grid_plan;
				grid_bag_1to2.close();
				grid_bag_2to1.close();

			}
		}

		move_grid(false);
		saved_path_name_list.clear();
		saved_path_name_list.push_back(saved_directory+"trajectory_bin_1_go.bag");
		//saved_path_name_list.push_back(saved_directory+"trajectory_2.bag");


		rosbag::Bag bag_1;
		moveit::planning_interface::MoveGroup::Plan tmp_plan;
		for(int i = 0;i < saved_path_name_list.size();i++){
			ROS_WARN_STREAM(saved_path_name_list[i]);
			bag_1.open(saved_path_name_list[i], rosbag::bagmode::Read);
			std::vector<std::string> topics;
			topics.push_back("/start_state");
			topics.push_back("/trajectory");
			topics.push_back("/planning_time");

			rosbag::View view_1(bag_1, rosbag::TopicQuery(topics));

			foreach(rosbag::MessageInstance const m, view_1)
    		{
    			
        		moveit_msgs::RobotState::Ptr s = m.instantiate<moveit_msgs::RobotState>();
        		if (s != NULL){ 
        			tmp_plan.start_state_ = *s;
        			ROS_WARN_STREAM(tmp_plan.start_state_.joint_state.position[0]<<","<<tmp_plan.start_state_.joint_state.position[1]);
        		}
        			moveit_msgs::RobotTrajectory::ConstPtr q = m.instantiate<moveit_msgs::RobotTrajectory>();
        		
        		if (q != NULL){
            		tmp_plan.trajectory_ = *q;
            	}
            		std_msgs::Float32::ConstPtr t = m.instantiate<std_msgs::Float32>();
        		
        		if (t != NULL)
        			tmp_plan.planning_time_ = t->data;

        		
    		}
    		ROS_WARN_STREAM("tmp_plan has "<<tmp_plan.trajectory_.joint_trajectory.points.size()<<" points");
    		saved_paths[saved_path_name_list[i]] = tmp_plan;
			bag_1.close();
		}
		saved_path_name_list.clear();
		saved_path_name_list.push_back(saved_directory+"trajectory_bin_1_back.bag");
			rosbag::Bag bag_2;
		for(int i = 0;i < saved_path_name_list.size();i++){
			ROS_WARN_STREAM(saved_path_name_list[i]);
			bag_2.open(saved_path_name_list[i], rosbag::bagmode::Read);
			std::vector<std::string> topics;
			topics.push_back("/start_state");
			topics.push_back("/trajectory");
			topics.push_back("/planning_time");

			rosbag::View view_2(bag_2, rosbag::TopicQuery(topics));

			foreach(rosbag::MessageInstance const m, view_2)
    		{
    			
        		moveit_msgs::RobotState::Ptr s = m.instantiate<moveit_msgs::RobotState>();
        		if (s != NULL){ 
        			tmp_plan.start_state_ = *s;
        			ROS_WARN_STREAM(tmp_plan.start_state_.joint_state.position[0]<<","<<tmp_plan.start_state_.joint_state.position[1]);
        		}
        			moveit_msgs::RobotTrajectory::ConstPtr q = m.instantiate<moveit_msgs::RobotTrajectory>();
        		
        		if (q != NULL){
            		tmp_plan.trajectory_ = *q;
            	}
            		std_msgs::Float32::ConstPtr t = m.instantiate<std_msgs::Float32>();
        		
        		if (t != NULL)
        			tmp_plan.planning_time_ = t->data;

        		
    		}
    		ROS_WARN_STREAM("tmp_plan has "<<tmp_plan.trajectory_.joint_trajectory.points.size()<<" points");
    		saved_paths[saved_path_name_list[i]] = tmp_plan;
			bag_2.close();
		}
		saved_path_name_list.clear();
		saved_path_name_list.push_back(saved_directory+"trajectory_bin_2_go.bag");
			rosbag::Bag bag_3;
		for(int i = 0;i < saved_path_name_list.size();i++){
			ROS_WARN_STREAM(saved_path_name_list[i]);
			bag_3.open(saved_path_name_list[i], rosbag::bagmode::Read);
			std::vector<std::string> topics;
			topics.push_back("/start_state");
			topics.push_back("/trajectory");
			topics.push_back("/planning_time");

			rosbag::View view_3(bag_3, rosbag::TopicQuery(topics));

			foreach(rosbag::MessageInstance const m, view_3)
    		{
    			
        		moveit_msgs::RobotState::Ptr s = m.instantiate<moveit_msgs::RobotState>();
        		if (s != NULL){ 
        			tmp_plan.start_state_ = *s;
        			ROS_WARN_STREAM(tmp_plan.start_state_.joint_state.position[0]<<","<<tmp_plan.start_state_.joint_state.position[1]);
        		}
        			moveit_msgs::RobotTrajectory::ConstPtr q = m.instantiate<moveit_msgs::RobotTrajectory>();
        		
        		if (q != NULL){
            		tmp_plan.trajectory_ = *q;
            	}
            		std_msgs::Float32::ConstPtr t = m.instantiate<std_msgs::Float32>();
        		
        		if (t != NULL)
        			tmp_plan.planning_time_ = t->data;

        		
    		}
    		ROS_WARN_STREAM("tmp_plan has "<<tmp_plan.trajectory_.joint_trajectory.points.size()<<" points");
    		saved_paths[saved_path_name_list[i]] = tmp_plan;
			bag_3.close();
		}

		saved_path_name_list.clear();
		saved_path_name_list.push_back(saved_directory+"trajectory_bin_2_back.bag");
			rosbag::Bag bag_4;
		for(int i = 0;i < saved_path_name_list.size();i++){
			ROS_WARN_STREAM(saved_path_name_list[i]);
			bag_4.open(saved_path_name_list[i], rosbag::bagmode::Read);
			std::vector<std::string> topics;
			topics.push_back("/start_state");
			topics.push_back("/trajectory");
			topics.push_back("/planning_time");

			rosbag::View view_4(bag_4, rosbag::TopicQuery(topics));

			foreach(rosbag::MessageInstance const m, view_4)
    		{
    			
        		moveit_msgs::RobotState::Ptr s = m.instantiate<moveit_msgs::RobotState>();
        		if (s != NULL){ 
        			tmp_plan.start_state_ = *s;
        			ROS_WARN_STREAM(tmp_plan.start_state_.joint_state.position[0]<<","<<tmp_plan.start_state_.joint_state.position[1]);
        		}
        			moveit_msgs::RobotTrajectory::ConstPtr q = m.instantiate<moveit_msgs::RobotTrajectory>();
        		
        		if (q != NULL){
            		tmp_plan.trajectory_ = *q;
            	}
            		std_msgs::Float32::ConstPtr t = m.instantiate<std_msgs::Float32>();
        		
        		if (t != NULL)
        			tmp_plan.planning_time_ = t->data;

        		
    		}
    		ROS_WARN_STREAM("tmp_plan has "<<tmp_plan.trajectory_.joint_trajectory.points.size()<<" points");
    		saved_paths[saved_path_name_list[i]] = tmp_plan;
			bag_4.close();
		}

	}

	bin_2_back_plan = saved_paths["trajectory_bin_2_back.bag"];
	bin_1_back_plan = saved_paths["trajectory_bin_1_back.bag"];
	bin_2_go_plan = saved_paths["trajectory_bin_2_go.bag"];
	bin_1_go_plan = saved_paths["trajectory_bin_1_go.bag"];

	// iiwa_msgs::JointPosition start_joint_position_1, start_joint_position_2; 
	// sensor_msgs::JointState starting_joint = transfer_plan_1.start_state_.joint_state;
	
	ROS_WARN_STREAM("INIT: print the starting position for stored plans");
	// start_joint_position_1.position = iiwa_ros::jointQuantityFromDouble(starting_joint.position[0],starting_joint.position[1],starting_joint.position[2],starting_joint.position[3],starting_joint.position[4],starting_joint.position[5],starting_joint.position[6]);  
	// ROS_WARN("plan_1");
	// printJointPosition(start_joint_position_1);
	// starting_joint = transfer_plan_2.start_state_.joint_state;
	// start_joint_position_2.position = iiwa_ros::jointQuantityFromDouble(starting_joint.position[0],starting_joint.position[1],starting_joint.position[2],starting_joint.position[3],starting_joint.position[4],starting_joint.position[5],starting_joint.position[6]);  
	// ROS_WARN("plan 2");
	// printJointPosition(start_joint_position_2);
	// ROS_WARN_STREAM("transfer_1 has "<<transfer_plan_1.trajectory_.joint_trajectory.points.size()<<" points");
	// ROS_WARN("pre-computed paths initiated!!!");

#endif	
}

void MoveItNode::test_joint_velocity(){
	// command_velocity_.velocity.a1 = 0.1;
	// command_velocity_.velocity.a2 = 0;
	// command_velocity_.velocity.a3 = 0;
	// command_velocity_.velocity.a4 = 0;
	// command_velocity_.velocity.a5 = 0;
	// command_velocity_.velocity.a6 = 0;
	// command_velocity_.velocity.a7 = 0;

	// my_iiwa.setJointVelocity(command_velocity_);
	// ros::Duration(3.0).sleep();
	// command_velocity_.velocity.a1 = 0;
	// my_iiwa.setJointVelocity(command_velocity_);
	//my_iiwa.getPathParametersService().setJointRelativeVelocity(0.5);
	group.setStartState(*group.getCurrentState());
	// group.setPoseTarget(transfer_pose_1);
group.setJointValueTarget(bin_1_go_joint);
	group.move();
	ros::Duration(3).sleep();
	group.setStartState(*group.getCurrentState());
	group.setJointValueTarget(bin_2_go_joint);
	// group.setPoseTarget(transfer_pose_2);
	group.setPlannerId("RRTstarkConfigDefault");
	group.plan(my_plan);
	PrintMotionPlan(my_plan);
	group.execute(my_plan);

}

void MoveItNode::PrintMotionPlan(moveit::planning_interface::MoveGroup::Plan plan)
{
	for(int i = 0; i < plan.trajectory_.joint_trajectory.points.size();i++)
	{
		ROS_WARN_STREAM(i<<" trajectory_point");
		std::cout<<std::endl;
		ROS_WARN_STREAM("position:");
		for(int j = 0; j < plan.trajectory_.joint_trajectory.points[i].positions.size(); j ++){
			std::cout<<plan.trajectory_.joint_trajectory.points[i].positions[j]<<",";
		}
		std::cout<<std::endl;

		ROS_WARN_STREAM("velocity:");
		for(int j = 0; j < plan.trajectory_.joint_trajectory.points[i].velocities.size(); j ++){
			std::cout<<plan.trajectory_.joint_trajectory.points[i].velocities[j]<<",";
		}
		std::cout<<std::endl;

		ROS_WARN_STREAM("accelerations:");
		for(int j = 0; j < plan.trajectory_.joint_trajectory.points[i].accelerations.size(); j ++){
			std::cout<<plan.trajectory_.joint_trajectory.points[i].accelerations[j]<<",";
		}
		std::cout<<std::endl;
		ROS_WARN_STREAM("time from start:"<<plan.trajectory_.joint_trajectory.points[i].time_from_start);
	}
}

bool MoveItNode::execute_trajectory(moveit::planning_interface::MoveGroup::Plan plan){
	bool success = false;
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_ac_("/iiwa/PositionJointInterface_trajectory_controller/follow_joint_trajectory", true);

	control_msgs::FollowJointTrajectoryActionGoal trajectory_goal;
	trajectory_ac_.waitForServer();

	trajectory_goal.goal.trajectory = plan.trajectory_.joint_trajectory;
	//trajectory_goal.goal.path_tolerance = 
	//trajectory_goal.goal.path
	trajectory_ac_.sendGoal(trajectory_goal.goal);
	success = trajectory_ac_.waitForResult();
	if(success){
		ROS_WARN_STREAM("actionlib: success");
	}else{
		ROS_WARN_STREAM("actionlib: failed");
	}
		auto result = trajectory_ac_.getResult();
		ROS_WARN_STREAM("execute_trajectory result:"<<result->error_code);
	
	return success;
}

void MoveItNode::pre_plan_grid(){
	char if_save;
	moveit::planning_interface::MoveItErrorCode error_code; 
	moveit::planning_interface::MoveGroup::Plan tmp_plan;
	std::tuple<int, int, int, int> tmp_tuple;
	int bin_total_id = 12;
	int bin_2_total_id = 12;
	int bin_1_total_id = 9;
	std::string resources_path = ros::package::getPath("chimp_resources");
    system(("rosparam load " +resources_path + "/config/grid_joint_config.yaml").c_str());
// read bin pose
    // read bin_1
    for(int i = 0; i < bin_2_total_id; i ++){
        std::vector<double>  bin_2_poses;
        node_handle.getParam("bin_2/id_"+std::to_string(i), bin_2_poses);  
      	bin_2_grid_map[i] = bin_2_poses;
    	
    }
	for(int i = 0; i < bin_1_total_id; i ++){
        std::vector<double> bin_1_poses;
        node_handle.getParam("bin_1/id_"+std::to_string(i), bin_1_poses);  
        ROS_WARN_STREAM(bin_1_poses[0]<<bin_1_poses[1]);
      	bin_1_grid_map[i] = bin_1_poses;
    	
    }

	for(int id_bin_1 = 0; id_bin_1 < bin_1_total_id; id_bin_1++){
		group.setStartState(*group.getCurrentState());
		group.setJointValueTarget(bin_1_grid_map[id_bin_1]);
		group.setPlannerId("RRTConnectkConfigDefault");
		group.move();
		for(int id_bin_2 = 0; id_bin_2 < bin_2_total_id; id_bin_2++){
			ROS_WARN_STREAM("from "<<id_bin_1<<" to "<<id_bin_2);
			group.setJointValueTarget(bin_2_grid_map[id_bin_2]);
			group.setStartState(*group.getCurrentState());
			//group.setPlannerId("RRTConnectkConfigDefault");
			//addGripperObject();

			//group.setPlannerId("RRTstarkConfigDefault");
			//group.setPlannerId("RRTConnectkConfigDefault");
			while(!(error_code= group.plan(tmp_plan))){} 

			//PrintMotionPlan(bin_2_go_plan);	
			
			ROS_WARN_STREAM("1 error code:"<<error_code.val);
			group.execute(tmp_plan); // from bin_1 to bin_2
			std::cout<<"\nPress y to save , n to skip...\n";
			//std::cin>>if_save;
			//if(if_save == 'y'){
			tmp_tuple = std::make_tuple(1,2,id_bin_1, id_bin_2);
			grid_saved_paths[tmp_tuple] = tmp_plan;
			ros::Duration(3).sleep();
			rosbag::Bag bag_forward;
			
    		if(grid_saved_paths.find(tmp_tuple) != grid_saved_paths.end()){ 
	   			std::string tmp_filename = "";
	   			tmp_filename += ( "path_1_to_2_" + std::to_string(id_bin_1) + "_" + std::to_string(id_bin_2) + ".bag"); 
	   			bag_forward.open(tmp_filename, rosbag::bagmode::Write);

			    moveit_msgs::RobotState start_state =  grid_saved_paths[tmp_tuple].start_state_;
			    moveit_msgs::RobotTrajectory trajectory = grid_saved_paths[tmp_tuple].trajectory_;
			    std_msgs::Float32 trajectory_time;
			    trajectory_time.data = grid_saved_paths[tmp_tuple].planning_time_;
			    ROS_WARN_STREAM(trajectory_time.data);
			    bag_forward.write("/start_state", ros::Time::now(), start_state);
			    bag_forward.write("/trajectory", ros::Time::now(), trajectory);
			    bag_forward.write("/planning_time", ros::Time::now(), trajectory_time);
			    bag_forward.close();
			}
			//}
			group.setJointValueTarget(bin_1_grid_map[id_bin_1]);
			group.setStartState(*group.getCurrentState());
			while(!(error_code= group.plan(tmp_plan))){} 
			ROS_WARN_STREAM("1 error code:"<<error_code.val);
			group.execute(tmp_plan); // from bin_1 to bin_2
			std::cout<<"\nPress y to save , n to skip...\n";
			//std::cin>>if_save;
			//if(if_save == 'y'){
			tmp_tuple = std::make_tuple(2,1,id_bin_2, id_bin_1);	
			grid_saved_paths[tmp_tuple] = tmp_plan;
			//}
			ros::Duration(3).sleep();
			rosbag::Bag bag_back;
			
    		if(grid_saved_paths.find(tmp_tuple) != grid_saved_paths.end()){ 
	   			std::string tmp_filename = "";
	   			tmp_filename += ( "path_2_to_1_" + std::to_string(id_bin_2) + "_" + std::to_string(id_bin_1) + ".bag"); 
	   			bag_back.open(tmp_filename, rosbag::bagmode::Write);

			    moveit_msgs::RobotState start_state =  grid_saved_paths[tmp_tuple].start_state_;
			    moveit_msgs::RobotTrajectory trajectory = grid_saved_paths[tmp_tuple].trajectory_;
			    std_msgs::Float32 trajectory_time;
			    trajectory_time.data = grid_saved_paths[tmp_tuple].planning_time_;
			    ROS_WARN_STREAM(trajectory_time.data);
			    bag_back.write("/start_state", ros::Time::now(), start_state);
			    bag_back.write("/trajectory", ros::Time::now(), trajectory);
			    bag_back.write("/planning_time", ros::Time::now(), trajectory_time);
			    bag_back.close();
			}
			
		}
	}
	while(1);
}

std::vector<double> MoveItNode::pre_plan(){
	char if_save;
	std::cout<<"\nPress n to end...\n";
	std::cin>>if_save;
	std::string saved_directory = "";
	saved_path_name_list.clear();
	saved_path_name_list.push_back(saved_directory+"trajectory_bin_1_go.bag");
	saved_path_name_list.push_back(saved_directory+"trajectory_bin_1_back.bag");
	saved_path_name_list.push_back(saved_directory+"trajectory_bin_2_go.bag");
	saved_path_name_list.push_back(saved_directory+"trajectory_bin_2_back.bag");
	moveit::planning_interface::MoveItErrorCode error_code; 
	group.setStartState(*group.getCurrentState());
	group.setJointValueTarget(bin_2_back_joint);
	group.setPlannerId("RRTConnectkConfigDefault");
	group.move();

	//bin_2_back_joint = group.getCurrentJointValues();
	ROS_WARN_STREAM("bin_2_back_joint:"<<bin_2_back_joint[0]<<","<<bin_2_back_joint[1]<<","<<bin_2_back_joint[2]<<","<<bin_2_back_joint[3]<<","<<bin_2_back_joint[4]<<","<<bin_2_back_joint[5]<<","<<bin_2_back_joint[6]);
	robot_state::RobotStatePtr transfer_state_1 = group.getCurrentState();
	//transfer_state = *transfer_state_1;
	
	group.setJointValueTarget(bin_2_go_joint);
	group.setStartState(*group.getCurrentState());
	//group.setPlannerId("RRTConnectkConfigDefault");
	//addGripperObject();

	group.setPlannerId("RRTstarkConfigDefault");
	//group.setPlannerId("RRTConnectkConfigDefault");
	while(!(error_code= group.plan(bin_2_go_plan))){} 

	PrintMotionPlan(bin_2_go_plan);	
	
	ROS_WARN_STREAM("2 error code:"<<error_code.val);
	group.execute(bin_2_go_plan); // from bin_1 to bin_2
	std::cout<<"\nPress y to save , n to skip...\n";
	std::cin>>if_save;
	if(if_save == 'y'){
		saved_paths[saved_path_name_list[2]] = bin_2_go_plan;
	}

	group.setJointValueTarget(bin_2_back_joint);
	group.setStartState(*group.getCurrentState());
	while(!(error_code= group.plan(bin_2_back_plan))){}   // from bin_2 to bin_1
	PrintMotionPlan(bin_2_back_plan);
	ROS_WARN_STREAM("3 error code:"<<error_code.val);
	// moveit_msgs::RobotState tmp_robotstate_msg = bin_2_back_plan.start_state_;
	// robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	// robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	// robot_state::RobotState tmp_robotstate(kinematic_model);
	// group.setPlannerId("RRTConnectkConfigDefault");
	// robotStateMsgToRobotState(tmp_robotstate_msg, tmp_robotstate);
	// group.setJointValueTarget(tmp_robotstate);
	// group.setStartState(*group.getCurrentState());
	// group.move();
	//bin_2_go_joint = group.getCurrentJointValues();
	ROS_WARN_STREAM("bin_2_go_joint:"<<bin_2_go_joint[0]<<","<<bin_2_go_joint[1]<<","<<bin_2_go_joint[2]<<","<<bin_2_go_joint[3]<<","<<bin_2_go_joint[4]<<","<<bin_2_go_joint[5]<<","<<bin_2_go_joint[6]);

	ros::Duration(2.0).sleep();
	group.setStartState(*group.getCurrentState());
	group.execute(bin_2_back_plan);
	std::cout<<"\nPress y to save , n to skip...\n";
	std::cin>>if_save;
	if(if_save == 'y'){
			saved_paths[saved_path_name_list[3]] = bin_2_back_plan;
	}


	group.setStartState(*group.getCurrentState());
	group.setJointValueTarget(bin_1_go_joint);
	group.setPlannerId("RRTConnectkConfigDefault");
	group.move();

	//bin_1_go_joint = group.getCurrentJointValues();
	ROS_WARN_STREAM("bin_1_go_joint:"<<bin_1_go_joint[0]<<","<<bin_1_go_joint[1]<<","<<bin_1_go_joint[2]<<","<<bin_1_go_joint[3]<<","<<bin_1_go_joint[4]<<","<<bin_1_go_joint[5]<<","<<bin_1_go_joint[6]);
	transfer_state_1 = group.getCurrentState();
	//transfer_state = *transfer_state_1;
	
	group.setJointValueTarget(bin_1_back_joint);
	group.setStartState(*group.getCurrentState());
	//group.setPlannerId("RRTConnectkConfigDefault");
	//addGripperObject();
	group.setPlannerId("RRTstarkConfigDefault");
	while(!(error_code= group.plan(bin_1_back_plan))){} 
	PrintMotionPlan(bin_1_back_plan);
	ROS_WARN_STREAM("2 error code:"<<error_code.val);
	group.execute(bin_1_back_plan); // from bin_1 to bin_2
	std::cout<<"\nPress y to save , n to skip...\n";
	std::cin>>if_save;
	if(if_save == 'y'){
			saved_paths[saved_path_name_list[1]] = bin_1_back_plan;
	}	
	group.setJointValueTarget(bin_1_go_joint);
	group.setStartState(*group.getCurrentState());
	while(!(error_code= group.plan(bin_1_go_plan))){}   // from bin_2 to bin_1
	PrintMotionPlan(bin_1_go_plan);
	ROS_WARN_STREAM("3 error code:"<<error_code.val);
	// tmp_robotstate_msg = bin_1_go_plan.start_state_;
	// robot_model_loader::RobotModelLoader robot_model_loader_2("robot_description");
	// robot_model::RobotModelPtr kinematic_model_2 = robot_model_loader_2.getModel();
	// robot_state::RobotState tmp_robotstate_2(kinematic_model_2);
	// group.setPlannerId("RRTConnectkConfigDefault");
	// robotStateMsgToRobotState(tmp_robotstate_msg, tmp_robotstate_2);
	// group.setJointValueTarget(tmp_robotstate_2);
	// group.setStartState(*group.getCurrentState());
	// group.move();
	//bin_1_back_joint = group.getCurrentJointValues();
	ROS_WARN_STREAM("bin_1_back_joint:"<<bin_1_back_joint[0]<<","<<bin_1_back_joint[1]<<","<<bin_1_back_joint[2]<<","<<bin_1_back_joint[3]<<","<<bin_1_back_joint[4]<<","<<bin_1_back_joint[5]<<","<<bin_1_back_joint[6]);

	ros::Duration(2.0).sleep();
	group.setStartState(*group.getCurrentState());
	group.execute(bin_1_go_plan);
	std::cout<<"\nPress y to save , n to skip...\n";
	std::cin>>if_save;
	if(if_save == 'y'){
				saved_paths[saved_path_name_list[0]] = bin_1_go_plan;
	}	
	ROS_WARN("pre_plan: finish go to home");
	group.setPlannerId("RRTConnectkConfigDefault");
	ros::Duration(2.0).sleep();
	goto_home();

	char wait_for_input;
	std::cout<<"\nPress n to end...\n";
	std::cin>>wait_for_input;
	

	// std::string saved_directory = "";
	// saved_path_name_list.clear();
	// saved_path_name_list.push_back(saved_directory+"trajectory_bin_1_go.bag");
	// saved_path_name_list.push_back(saved_directory+"trajectory_bin_1_back.bag");
	// saved_path_name_list.push_back(saved_directory+"trajectory_bin_2_go.bag");
	// saved_path_name_list.push_back(saved_directory+"trajectory_bin_2_back.bag");

	// saved_paths[saved_path_name_list[0]] = bin_1_go_plan;
	// saved_paths[saved_path_name_list[1]] = bin_1_back_plan;
	// saved_paths[saved_path_name_list[2]] = bin_2_go_plan;
	// saved_paths[saved_path_name_list[3]] = bin_2_back_plan;
	//save the motion plans
	rosbag::Bag bag;
	for(int i = 0; i < saved_path_name_list.size();i ++){ 
	    if(saved_paths.find(saved_path_name_list[i]) != saved_paths.end()){ 
		    bag.open(saved_path_name_list[i], rosbag::bagmode::Write);

		    moveit_msgs::RobotState start_state =  saved_paths[saved_path_name_list[i]].start_state_;
		    moveit_msgs::RobotTrajectory trajectory = saved_paths[saved_path_name_list[i]].trajectory_;
		    std_msgs::Float32 trajectory_time;
		    trajectory_time.data = saved_paths[saved_path_name_list[i]].planning_time_;
		    ROS_WARN_STREAM(trajectory_time.data);
		    bag.write("/start_state", ros::Time::now(), start_state);
		    bag.write("/trajectory", ros::Time::now(), trajectory);
		    bag.write("/planning_time", ros::Time::now(), trajectory_time);
		    bag.close();
		}
	}
    //pre_plan = true;
	return bin_2_back_joint;






// 	bool transfer_1 = false;
// 	bool transfer_2 = false;
// 	// This will create a JointQuantity will seven zeros
// 	moveit::planning_interface::MoveItErrorCode error_code; 
// 	group.setStartState(*group.getCurrentState());
// 	group.setPoseTarget(transfer_pose_1);
// 	group.move();
// 	//
// 	utilities::sleepForMotion(my_iiwa,5.0);
// 	//my_iiwa.setJointPosition(my_joint_position);
// 	//robot_state::RobotState transfer_state;
// 	transfer_joint_1 = group.getCurrentJointValues();
// 	ROS_WARN_STREAM(transfer_joint_1[0]<<","<<transfer_joint_1[1]<<","<<transfer_joint_1[2]<<","<<transfer_joint_1[3]<<","<<transfer_joint_1[4]<<","<<transfer_joint_1[5]<<","<<transfer_joint_1[6]);
// 	robot_state::RobotStatePtr transfer_state_1 = group.getCurrentState();
// 	//transfer_state = *transfer_state_1;
	
// 	group.setPoseTarget(transfer_pose_2);
// 	group.setStartState(*group.getCurrentState());
// 	//group.setPlannerId("RRTConnectkConfigDefault");
// 	//addGripperObject();
// 	group.setPlannerId("RRTstarkConfigDefault");
// 	while(!(error_code= group.plan(transfer_plan_1))){} 



// 	ROS_WARN_STREAM("2 error code:"<<error_code.val);
// 	group.execute(transfer_plan_1); // from bin_1 to bin_2
// 	//group.move();
// 	ros::Duration(10.0).sleep();
// 	robot_state::RobotStatePtr transfer_state_2 = group.getCurrentState();
// 	//group.setPoseTarget(transfer_pose_1);
	
// 	group.setJointValueTarget(transfer_joint_1);
// 	//group.setStartState(*group.getCurrentState());
// 	group.setStartState(*group.getCurrentState());
// 	while(!(error_code= group.plan(transfer_plan_2))){}   // from bin_2 to bin_1
// 	ROS_WARN_STREAM("3 error code:"<<error_code.val);
// 	moveit_msgs::RobotState tmp_robotstate_msg = transfer_plan_2.start_state_;
// 	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
// robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
// 	robot_state::RobotState tmp_robotstate(kinematic_model);
// group.setPlannerId("RRTConnectkConfigDefault");
// 	robotStateMsgToRobotState(tmp_robotstate_msg, tmp_robotstate);
// 	group.setJointValueTarget(tmp_robotstate);
// 	group.setStartState(*group.getCurrentState());
// 	group.move();
// 	transfer_joint_2 = group.getCurrentJointValues();
// 	ROS_WARN_STREAM(transfer_joint_2[0]<<","<<transfer_joint_2[1]<<","<<transfer_joint_2[2]<<","<<transfer_joint_2[3]<<","<<transfer_joint_2[4]<<","<<transfer_joint_2[5]<<","<<transfer_joint_2[6]);

// 	ros::Duration(2.0).sleep();
// 	group.setStartState(*group.getCurrentState());
// 	group.execute(transfer_plan_2);


// 	ros::Duration(2.0).sleep();
	// ROS_WARN("pre_plan: finish go to home");

	// ros::Duration(2.0).sleep();
	// goto_home();

	// char wait_for_input;
	// std::cout<<"\nPress n to end...\n";
	// std::cin>>wait_for_input;
	

	// std::string saved_directory = "";
	// saved_path_name_list.clear();
	// saved_path_name_list.push_back(saved_directory+"trajectory_1.bag");
	// saved_path_name_list.push_back(saved_directory+"trajectory_2.bag");
	// saved_paths[saved_path_name_list[0]] = transfer_plan_1;
	// saved_paths[saved_path_name_list[1]] = transfer_plan_2;
	// //save the motion plans
	// rosbag::Bag bag;
	// for(int i = 0; i < saved_path_name_list.size();i ++){ 
	//     bag.open(saved_path_name_list[i], rosbag::bagmode::Write);

	//     moveit_msgs::RobotState start_state =  saved_paths[saved_path_name_list[i]].start_state_;
	//     moveit_msgs::RobotTrajectory trajectory = saved_paths[saved_path_name_list[i]].trajectory_;
	//     std_msgs::Float32 trajectory_time;
	//     trajectory_time.data = saved_paths[saved_path_name_list[i]].planning_time_;
	//     ROS_WARN_STREAM(trajectory_time.data);
	//     bag.write("/start_state", ros::Time::now(), start_state);
	//     bag.write("/trajectory", ros::Time::now(), trajectory);
	//     bag.write("/planning_time", ros::Time::now(), trajectory_time);
	//     bag.close();
	// }
 //    //pre_plan = true;
	// return transfer_joint_1;

}



void MoveItNode::addCollisionObjectFromMesh(moveit_msgs::CollisionObject& colObj, std::string meshFile, std::vector<float> objPose, std::string bin_id)
{
	// ROS_INFO_STREAM(collision_objects.size());
	ROS_INFO("Adding collision object from mesh file!");


	// shapes::Mesh* m = shapes::createMeshFromResource("package://motion_planning/models/obj.obj");
	shapes::Mesh* m = shapes::createMeshFromResource(meshFile);
	shape_msgs::Mesh obj_mesh;
	shapes::ShapeMsg obj_mesh_msg;
	shapes::constructMsgFromShape(m,obj_mesh_msg);
	obj_mesh = boost::get<shape_msgs::Mesh>(obj_mesh_msg);

	colObj.id = bin_id;
	colObj.header.frame_id = group.getPlanningFrame();
	colObj.meshes.push_back(obj_mesh);
	geometry_msgs::Pose obj_pose;
	obj_pose.position.x = objPose.at(0);
	obj_pose.position.y = objPose.at(1);
	obj_pose.position.z = objPose.at(2);
	obj_pose.orientation.x = objPose.at(3);
	obj_pose.orientation.y = objPose.at(4);
	obj_pose.orientation.z = objPose.at(5);
	obj_pose.orientation.w = objPose.at(6);
	colObj.mesh_poses.push_back(obj_pose);

	colObj.operation = colObj.ADD;
}





void MoveItNode::addCollisionObject(moveit_msgs::CollisionObject& colObj, std::string objId, std::vector<float> objDim, std::vector<float> objPose){
	colObj.id = objId;
	colObj.header.frame_id = group.getPlanningFrame();

	shape_msgs::SolidPrimitive objPrimitive;

	if (objId.compare("table_round")==0)
	{
		objPrimitive.type = objPrimitive.CYLINDER;
		objPrimitive.dimensions.resize(2);
		objPrimitive.dimensions[0] = objDim[0];		
		objPrimitive.dimensions[1] = objDim[1];
	}
	else
	{
		objPrimitive.type = objPrimitive.BOX;
		objPrimitive.dimensions.resize(3);
		objPrimitive.dimensions[0] = objDim[0];
		objPrimitive.dimensions[1] = objDim[1];
		objPrimitive.dimensions[2] = objDim[2];
	}

	geometry_msgs::Pose objectPose;
	objectPose.position.x = objPose[0];
	objectPose.position.y = objPose[1];
	objectPose.position.z = objPose[2];
	objectPose.orientation.x = objPose[3];
	objectPose.orientation.y = objPose[4];
	objectPose.orientation.z = objPose[5];
	objectPose.orientation.w = objPose[6];

	colObj.primitives.push_back(objPrimitive);
	colObj.primitive_poses.push_back(objectPose);
	colObj.operation = colObj.ADD;

	ROS_INFO("Added the object %s as a collision object.",objId.c_str());
}


bool MoveItNode::planToStartOfPreC(std::string source_bin, std::string target_bin, geometry_msgs::Pose target_pose, int mode)
{
	bool success = false;
	
	

	geometry_msgs::PoseStamped source_state_ps;
	geometry_msgs::Pose source_state_pose, target_state_pose;

	my_iiwa.getCartesianPose(source_state_ps);
	source_state_pose = source_state_ps.pose;
	ROS_WARN_STREAM("CURRENT POSE:"<<source_state_pose.position.x<<","<<source_state_pose.position.y<<","<<source_state_pose.position.z);
	int source_index = closest_index(source_state_pose, source_bin);

	target_state_pose = target_pose;
	int target_index = closest_index(target_state_pose, target_bin);

	ROS_ERROR_STREAM("\n\n\n  PRECOMPUTATION IS FROM: "<<source_bin<<": "<<source_index<<" -> "<<target_bin<<": "<<target_index);

	int source_bin_id = 1;
	int target_bin_id = 1;

	if(source_bin == "bin_2")
		source_bin_id = 2;
	if(target_bin == "bin_2")
		target_bin_id = 2;

	moveit::planning_interface::MoveGroup::Plan current_plan;

	current_plan = grid_saved_paths[std::make_tuple(source_bin_id,target_bin_id, source_index, target_index)];



	sensor_msgs::JointState starting_joint = current_plan.start_state_.joint_state;
	moveit_msgs::RobotTrajectory final_robot_trajectory = current_plan.trajectory_; //.joint_trajectory.points.back;
	trajectory_msgs::JointTrajectory final_joint_trajectory = final_robot_trajectory.joint_trajectory;
	int trajectory_length = current_plan.trajectory_.joint_trajectory.points.size();
	trajectory_msgs::JointTrajectoryPoint final_point = final_joint_trajectory.points[trajectory_length-1];
	
	std::vector<double> start_joint_position;
	iiwa_msgs::JointPosition my_joint_position, last_joint_position; 
	last_joint_position.position.a1 = final_point.positions[0];
	last_joint_position.position.a2 = final_point.positions[1];
	last_joint_position.position.a3 = final_point.positions[2];
	last_joint_position.position.a4 = final_point.positions[3];
	last_joint_position.position.a5 = final_point.positions[4];
	last_joint_position.position.a6 = final_point.positions[5];
	last_joint_position.position.a7 = final_point.positions[6];
	
	// = iiwa_ros::jointQuantityFromMultipleDouble(final_point.positions[0],final_point.positions[1],final_point.positions[2],final_point.positions[3],final_point.positions[4],final_point.positions[5],final_point.positions[6]);  
	my_joint_position.position.a1 =  starting_joint.position[0];
	my_joint_position.position.a2 =  starting_joint.position[1];
	my_joint_position.position.a3 =  starting_joint.position[2];
	my_joint_position.position.a4 =  starting_joint.position[3];
	my_joint_position.position.a5 =  starting_joint.position[4];
	my_joint_position.position.a6 =  starting_joint.position[5];
	my_joint_position.position.a7 =  starting_joint.position[6];

	// = iiwa_ros::jointQuantityFromMultipleDouble(starting_joint.position[0],starting_joint.position[1],starting_joint.position[2],starting_joint.position[3],starting_joint.position[4],starting_joint.position[5],starting_joint.position[6]);  
	ROS_WARN_STREAM("starting joint position for transfer plan 2");
	printJointPosition(my_joint_position);
	ROS_INFO("before steering");
	my_iiwa.getJointPosition(current_joint_position_);

	printJointPosition(current_joint_position_);
	start_joint_position = {my_joint_position.position.a1,my_joint_position.position.a2,my_joint_position.position.a3,my_joint_position.position.a4,my_joint_position.position.a5,my_joint_position.position.a6,my_joint_position.position.a7};
	group.setJointValueTarget(start_joint_position);
	group.setPlannerId("RRTConnectkConfigDefault");
	success = group.move();

	return success;

}



//mode
//0: from pre_grasp to drop(higher to lower)
//1: from drop to pre_grasp(lower to higher)
bool MoveItNode::planToStartOfPreC(std::string source_bin, std::string target_bin, int mode)
{
	bool success = false;
	
	moveit::planning_interface::MoveGroup::Plan current_plan;
	if((source_bin == "bin_1") && (target_bin == "bin_2") && mode == 1){  
		ROS_WARN_STREAM("execute bin_2_go_plan");        
		current_plan = bin_2_go_plan;
	}else if((source_bin == "bin_2") && (target_bin == "bin_1") && mode == 1){           
		ROS_WARN_STREAM("bin_1_go_plan");
		current_plan = bin_1_go_plan;
	}else if((source_bin == "bin_1") && (target_bin == "bin_2") && mode == 0){
		ROS_WARN_STREAM("bin_1_back_plan");
		current_plan = bin_1_back_plan;
	}else if((source_bin == "bin_2") && (target_bin == "bin_1") && mode == 0){
		ROS_WARN_STREAM("bin_2_back_plan");
		current_plan = bin_2_back_plan;
	}

	sensor_msgs::JointState starting_joint = current_plan.start_state_.joint_state;
	moveit_msgs::RobotTrajectory final_robot_trajectory = current_plan.trajectory_; //.joint_trajectory.points.back;
	trajectory_msgs::JointTrajectory final_joint_trajectory = final_robot_trajectory.joint_trajectory;
	int trajectory_length = current_plan.trajectory_.joint_trajectory.points.size();
	trajectory_msgs::JointTrajectoryPoint final_point = final_joint_trajectory.points[trajectory_length-1];
	
	std::vector<double> start_joint_position;
	iiwa_msgs::JointPosition my_joint_position, last_joint_position; 
	last_joint_position.position.a1 = final_point.positions[0];
	last_joint_position.position.a2 = final_point.positions[1];
	last_joint_position.position.a3 = final_point.positions[2];
	last_joint_position.position.a4 = final_point.positions[3];
	last_joint_position.position.a5 = final_point.positions[4];
	last_joint_position.position.a6 = final_point.positions[5];
	last_joint_position.position.a7 = final_point.positions[6];
	
	// = iiwa_ros::jointQuantityFromMultipleDouble(final_point.positions[0],final_point.positions[1],final_point.positions[2],final_point.positions[3],final_point.positions[4],final_point.positions[5],final_point.positions[6]);  
	my_joint_position.position.a1 =  starting_joint.position[0];
	my_joint_position.position.a2 =  starting_joint.position[1];
	my_joint_position.position.a3 =  starting_joint.position[2];
	my_joint_position.position.a4 =  starting_joint.position[3];
	my_joint_position.position.a5 =  starting_joint.position[4];
	my_joint_position.position.a6 =  starting_joint.position[5];
	my_joint_position.position.a7 =  starting_joint.position[6];

	// = iiwa_ros::jointQuantityFromMultipleDouble(starting_joint.position[0],starting_joint.position[1],starting_joint.position[2],starting_joint.position[3],starting_joint.position[4],starting_joint.position[5],starting_joint.position[6]);  
	ROS_WARN_STREAM("starting joint position for transfer plan 2");
	printJointPosition(my_joint_position);
	ROS_INFO("before steering");
	my_iiwa.getJointPosition(current_joint_position_);

	printJointPosition(current_joint_position_);
	start_joint_position = {my_joint_position.position.a1,my_joint_position.position.a2,my_joint_position.position.a3,my_joint_position.position.a4,my_joint_position.position.a5,my_joint_position.position.a6,my_joint_position.position.a7};
	group.setJointValueTarget(start_joint_position);
	group.setPlannerId("RRTConnectkConfigDefault");
	success = group.move();

	return success;

}
// bool MoveitNode::executeGridMove(){

// }


bool MoveItNode::execute(std::string source_bin, std::string target_bin, int mode){
	bool success = false;
	
	moveit::planning_interface::MoveGroup::Plan current_plan;
	if((source_bin == "bin_1") && (target_bin == "bin_2") && mode == 1){  
		ROS_WARN_STREAM("execute bin_2_go_plan");        
		current_plan = bin_2_go_plan;
	}else if((source_bin == "bin_2") && (target_bin == "bin_1") && mode == 1){           
		ROS_WARN_STREAM("bin_1_go_plan");
		current_plan = bin_1_go_plan;
	}else if((source_bin == "bin_1") && (target_bin == "bin_2") && mode == 0){
		ROS_WARN_STREAM("bin_1_back_plan");
		current_plan = bin_1_back_plan;
	}else if((source_bin == "bin_2") && (target_bin == "bin_1") && mode == 0){
		ROS_WARN_STREAM("bin_2_back_plan");
		current_plan = bin_2_back_plan;
	}

	sensor_msgs::JointState starting_joint = current_plan.start_state_.joint_state;
	moveit_msgs::RobotTrajectory final_robot_trajectory = current_plan.trajectory_; //.joint_trajectory.points.back;
	trajectory_msgs::JointTrajectory final_joint_trajectory = final_robot_trajectory.joint_trajectory;
	int trajectory_length = current_plan.trajectory_.joint_trajectory.points.size();
	trajectory_msgs::JointTrajectoryPoint final_point = final_joint_trajectory.points[trajectory_length-1];
	
	std::vector<double> start_joint_position;
	iiwa_msgs::JointPosition my_joint_position, last_joint_position; 
	last_joint_position.position.a1 = final_point.positions[0];
	last_joint_position.position.a2 = final_point.positions[1];
	last_joint_position.position.a3 = final_point.positions[2];
	last_joint_position.position.a4 = final_point.positions[3];
	last_joint_position.position.a5 = final_point.positions[4];
	last_joint_position.position.a6 = final_point.positions[5];
	last_joint_position.position.a7 = final_point.positions[6];
	
	// = iiwa_ros::jointQuantityFromMultipleDouble(final_point.positions[0],final_point.positions[1],final_point.positions[2],final_point.positions[3],final_point.positions[4],final_point.positions[5],final_point.positions[6]);  
	my_joint_position.position.a1 =  starting_joint.position[0];
	my_joint_position.position.a2 =  starting_joint.position[1];
	my_joint_position.position.a3 =  starting_joint.position[2];
	my_joint_position.position.a4 =  starting_joint.position[3];
	my_joint_position.position.a5 =  starting_joint.position[4];
	my_joint_position.position.a6 =  starting_joint.position[5];
	my_joint_position.position.a7 =  starting_joint.position[6];

	// = iiwa_ros::jointQuantityFromMultipleDouble(starting_joint.position[0],starting_joint.position[1],starting_joint.position[2],starting_joint.position[3],starting_joint.position[4],starting_joint.position[5],starting_joint.position[6]);  
	ROS_WARN_STREAM("starting joint position for transfer plan 2");
	printJointPosition(my_joint_position);
	ROS_INFO("before steering");
	my_iiwa.getJointPosition(current_joint_position_);

	printJointPosition(current_joint_position_);
	start_joint_position = {my_joint_position.position.a1,my_joint_position.position.a2,my_joint_position.position.a3,my_joint_position.position.a4,my_joint_position.position.a5,my_joint_position.position.a6,my_joint_position.position.a7};
	group.setJointValueTarget(start_joint_position);
	group.setPlannerId("RRTConnectkConfigDefault");
	group.move();
	my_iiwa.setJointPosition(my_joint_position);
	// moveit_msgs::RobotState tmp_robotstate_msg = transfer_plan_2.start_state_;
	// robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	// robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	// robot_state::RobotState tmp_robotstate(kinematic_model);

	// robotStateMsgToRobotState(tmp_robotstate_msg, tmp_robotstate);
	// group.setJointValueTarget(tmp_robotstate);
	// group.setStartState(*group.getCurrentState());
	// group.move();
	//ros::Duration(1.0).sleep();
	utilities::sleepForMotion(my_iiwa,0.2);
	ROS_INFO("starting point:");
	printJointPosition(my_joint_position);
	my_iiwa.getJointPosition(current_joint_position_);

	printJointPosition(current_joint_position_);
	group.setStartState(*group.getCurrentState());
	ROS_WARN_STREAM("transfer_2 has "<<current_plan.trajectory_.joint_trajectory.points.size()<<" points");
	// moveit::planning_interface::MoveItErrorCode second_code = group.execute(transfer_plan_2);
	// utilities::sleepForMotion(my_iiwa,0.2);
	// //ros::Duration(1.0).sleep();
	// ROS_WARN_STREAM("transfer_2 error code:"<<second_code.val);
	success = execute_trajectory(current_plan);
	my_iiwa.getJointPosition(current_joint_position_);
	ROS_INFO("final point:");
	printJointPosition(current_joint_position_);
	//my_iiwa.setJointPosition(last_joint_position);
	//utilities::sleepForMotion(my_iiwa,2.0);
	printJointPosition(last_joint_position);
	
	
	return success;
}

double MoveItNode::distance_between_poses(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
	double x1 = p1.position.x;
	double y1 = p1.position.y;
	double z1 = p1.position.z;
	double x2 = p2.position.x;
	double y2 = p2.position.y;
	double z2 = p2.position.z;

	return std::sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1) );
}


int MoveItNode::closest_index(geometry_msgs::Pose p, std::string bin)
{
	auto pose_map = bin_1_id_pose_map;

	if(bin == "bin_2")
		pose_map = bin_2_id_pose_map;

	int min_index = -1;
	double min_dist = 100000;
	for(auto pm: pose_map)
	{
		double dist = distance_between_poses(p, pm.second);
		if(dist<min_dist)
		{
			min_dist = dist;
			min_index = pm.first;
		}
	}

	ROS_WARN_STREAM("Min Index and distance: "<<min_index<<", "<<min_dist);
	return min_index;
}




bool MoveItNode::execute(std::string source_bin, std::string target_bin, geometry_msgs::Pose target_pose, int mode){
	bool success = false;
	

	geometry_msgs::PoseStamped source_state_ps;
	geometry_msgs::Pose source_state_pose, target_state_pose;

	my_iiwa.getCartesianPose(source_state_ps);
	source_state_pose = source_state_ps.pose;
	int source_index = closest_index(source_state_pose, source_bin);

	target_state_pose = target_pose;
	int target_index = closest_index(target_state_pose, target_bin);

	ROS_ERROR_STREAM("\n\n\n  PRECOMPUTATION IS FROM: "<<source_bin<<": "<<source_index<<" -> "<<target_bin<<": "<<target_index);

	int source_bin_id = 1;
	int target_bin_id = 1;

	if(source_bin == "bin_2")
		source_bin_id = 2;
	if(target_bin == "bin_2")
		target_bin_id = 2;




	moveit::planning_interface::MoveGroup::Plan current_plan;
	

	current_plan = grid_saved_paths[std::make_tuple(source_bin_id,target_bin_id, source_index, target_index)];

	sensor_msgs::JointState starting_joint = current_plan.start_state_.joint_state;
	moveit_msgs::RobotTrajectory final_robot_trajectory = current_plan.trajectory_; //.joint_trajectory.points.back;
	trajectory_msgs::JointTrajectory final_joint_trajectory = final_robot_trajectory.joint_trajectory;
	int trajectory_length = current_plan.trajectory_.joint_trajectory.points.size();
	trajectory_msgs::JointTrajectoryPoint final_point = final_joint_trajectory.points[trajectory_length-1];
	
	std::vector<double> start_joint_position;
	iiwa_msgs::JointPosition my_joint_position, last_joint_position; 
	last_joint_position.position.a1 = final_point.positions[0];
	last_joint_position.position.a2 = final_point.positions[1];
	last_joint_position.position.a3 = final_point.positions[2];
	last_joint_position.position.a4 = final_point.positions[3];
	last_joint_position.position.a5 = final_point.positions[4];
	last_joint_position.position.a6 = final_point.positions[5];
	last_joint_position.position.a7 = final_point.positions[6];
	
	// = iiwa_ros::jointQuantityFromMultipleDouble(final_point.positions[0],final_point.positions[1],final_point.positions[2],final_point.positions[3],final_point.positions[4],final_point.positions[5],final_point.positions[6]);  
	my_joint_position.position.a1 =  starting_joint.position[0];
	my_joint_position.position.a2 =  starting_joint.position[1];
	my_joint_position.position.a3 =  starting_joint.position[2];
	my_joint_position.position.a4 =  starting_joint.position[3];
	my_joint_position.position.a5 =  starting_joint.position[4];
	my_joint_position.position.a6 =  starting_joint.position[5];
	my_joint_position.position.a7 =  starting_joint.position[6];

	// = iiwa_ros::jointQuantityFromMultipleDouble(starting_joint.position[0],starting_joint.position[1],starting_joint.position[2],starting_joint.position[3],starting_joint.position[4],starting_joint.position[5],starting_joint.position[6]);  
	//ROS_WARN_STREAM("starting joint position for transfer plan 2");
	printJointPosition(my_joint_position);
	//ROS_INFO("before steering");
	my_iiwa.getJointPosition(current_joint_position_);

	printJointPosition(current_joint_position_);
	start_joint_position = {my_joint_position.position.a1,my_joint_position.position.a2,my_joint_position.position.a3,my_joint_position.position.a4,my_joint_position.position.a5,my_joint_position.position.a6,my_joint_position.position.a7};
	group.setJointValueTarget(start_joint_position);
	group.setPlannerId("RRTConnectkConfigDefault");
	group.move();
	my_iiwa.setJointPosition(my_joint_position);
	// moveit_msgs::RobotState tmp_robotstate_msg = transfer_plan_2.start_state_;
	// robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	// robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	// robot_state::RobotState tmp_robotstate(kinematic_model);

	// robotStateMsgToRobotState(tmp_robotstate_msg, tmp_robotstate);
	// group.setJointValueTarget(tmp_robotstate);
	// group.setStartState(*group.getCurrentState());
	// group.move();
	//ros::Duration(1.0).sleep();
	utilities::sleepForMotion(my_iiwa,0.2);
	//ROS_INFO("starting point:");
	printJointPosition(my_joint_position);
	my_iiwa.getJointPosition(current_joint_position_);

	printJointPosition(current_joint_position_);
	group.setStartState(*group.getCurrentState());
	//ROS_WARN_STREAM("transfer_2 has "<<current_plan.trajectory_.joint_trajectory.points.size()<<" points");
	// moveit::planning_interface::MoveItErrorCode second_code = group.execute(transfer_plan_2);
	// utilities::sleepForMotion(my_iiwa,0.2);
	// //ros::Duration(1.0).sleep();
	// ROS_WARN_STREAM("transfer_2 error code:"<<second_code.val);
	// int space;
	// std::cin>>space;
	success = execute_trajectory(current_plan);
	// std::cin>>space;
	my_iiwa.getJointPosition(current_joint_position_);
	//ROS_INFO("final point:");
	printJointPosition(current_joint_position_);
	//my_iiwa.setJointPosition(last_joint_position);
	//utilities::sleepForMotion(my_iiwa,2.0);
	printJointPosition(last_joint_position);
	
	
	return success;
}




void MoveItNode::test_grid_move(){
	int id_end;
	int id_start;
	for(int i = 0; i < 4; i ++){
		for(int j = 0; j < 4; j ++){
			id_start = 4*j + i;
			for(int k = 0; k < 4;k ++){
				for(int p = 0;p < 4; p ++){
					id_end = 4*p + k;
					//plan_and_execute(bin_2_id_pose_map[id_start]);
					ROS_WARN_STREAM("bin_2_id_pose_map["<<id_start<<"]:"<<bin_2_id_pose_map[id_start].position.x<<","<<bin_2_id_pose_map[id_start].position.y);
					if(execute("bin_2", "bin_1", bin_1_id_pose_map[id_end], 0) == false){
						ROS_WARN_STREAM("bin_2 to bin_1 "<<id_start<<" to "<<id_end<<" failed");
					}else{
						ROS_WARN_STREAM("bin_2 to bin_1 "<<id_start<<" to "<<id_end<<" succeed");
					}
					//plan_and_execute(bin_1_id_pose_map[id_end]);
					ROS_WARN_STREAM("bin_1_id_pose_map["<<id_end<<"]:"<<bin_1_id_pose_map[id_end].position.x<<","<<bin_1_id_pose_map[id_end].position.y);
					if(execute("bin_1","bin_2", bin_2_id_pose_map[id_start], 0) == false){ 
						ROS_WARN_STREAM("bin_1 to bin_2 "<<id_end<<" to "<<id_start<<" failed");
					}else{
						ROS_WARN_STREAM("bin_1 to bin_2 "<<id_end<<" to "<<id_start<<" succeed");
					}

				}
			}
		}
	}
}

void MoveItNode::move_grid(bool is_real){
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
			bin_2_id_pose_map[id] = temp;
			
			ROS_WARN_STREAM("id_"<<id<<": ["<<bin_2_id_pose_map[id].position.x<<","<<bin_2_id_pose_map[id].position.y<<","<<bin_2_id_pose_map[id].position.z<<"]");
		}

	}
	ROS_WARN("precomputed: bin_1 position");
	for(int i = 0; i < 3; i ++){
		for(int j = 0; j < 3; j ++){
			int id = 3*i + j;
			geometry_msgs::Pose temp;
			temp = bin_1_center;
			temp.position.x = bin_1_center.position.x + bin_1_x_axis.first*bin_1_id_relative_pose_map[id].first + bin_1_y_axis.first * bin_1_id_relative_pose_map[id].second;
			temp.position.y = bin_1_center.position.y + bin_1_x_axis.second*bin_1_id_relative_pose_map[id].first + bin_1_y_axis.second * bin_1_id_relative_pose_map[id].second; 
			bin_1_id_pose_map[id] = temp;
			ROS_WARN_STREAM("id_"<<id<<": ["<<bin_1_id_pose_map[id].position.x<<","<<bin_1_id_pose_map[id].position.y<<","<<bin_1_id_pose_map[id].position.z<<"]");
		}

	}

	bool rewrite_yaml = true;
	//  
	std::ofstream myfile;
 	myfile.open ("/home/cm1074/kuka_ws/src/chimp_resources/config/grid_joint_config.yaml", std::ios::out | std::ios::trunc);
  	//myfile << "Writing this to a file.\n";
	if(is_real){
		myfile << "bin_1:\n";
		for(int i = 0; i < 3; i ++){
			for(int j = 0; j < 3; j ++){
				int id = 3*i + j;
				ROS_WARN_STREAM("Moving to id:"<<id<<"---"<<bin_1_id_pose_map[id].position.x<<","<<bin_1_id_pose_map[id].position.y<<","<<bin_1_id_pose_map[id].position.z);
				group.setJointValueTarget(bin_2_back_joint);
				group.setPlannerId("RRTConnectkConfigDefault");
				group.move();
				geometry_msgs::PoseStamped target;
				target.pose = bin_1_id_pose_map[id];
				plan_and_execute_via_waypoints(target);
				//group.setPoseTarget(bin_1_id_pose_map[id]);
				publish_cylinder(bin_1_id_pose_map[id]);
				//group.move();
				my_iiwa.getJointPosition(current_joint_position_);
				ros::Duration(3).sleep();
				printJointPosition(current_joint_position_);
				if(rewrite_yaml){
					myfile << "id_"<<id<<": ["<<current_joint_position_.position.a1<<","<<current_joint_position_.position.a2<<","<<current_joint_position_.position.a3<<","<<current_joint_position_.position.a4<<","<<current_joint_position_.position.a5<<","<<current_joint_position_.position.a6<<","<<current_joint_position_.position.a7<<"]\n"; 
				}
			}
		} 
		group.setJointValueTarget(bin_2_go_joint);
		group.setPlannerId("RRTConnectkConfigDefault");
		group.move();
		myfile << "bin_2:\n";
		for(int j = 0; j < 4; j ++){
		for(int i = 0; i < 3; i ++){
			int id = 3*j + i;
				ROS_WARN_STREAM("Moving to id:"<<id<<"---"<<bin_2_id_pose_map[id].position.x<<","<<bin_2_id_pose_map[id].position.y<<","<<bin_2_id_pose_map[id].position.z);
				group.setJointValueTarget(bin_2_go_joint);
				group.setPlannerId("RRTConnectkConfigDefault");
				group.move();
				geometry_msgs::PoseStamped target;
				target.pose = bin_2_id_pose_map[id];
				plan_and_execute_via_waypoints(target);
				//group.setPoseTarget(bin_2_id_pose_map[id]);
				publish_cylinder(bin_2_id_pose_map[id]);
				//group.move();
				my_iiwa.getJointPosition(current_joint_position_);
				ros::Duration(3).sleep();
				printJointPosition(current_joint_position_);
				if(rewrite_yaml){
					myfile << "id_"<<id<<": ["<<current_joint_position_.position.a1<<","<<current_joint_position_.position.a2<<","<<current_joint_position_.position.a3<<","<<current_joint_position_.position.a4<<","<<current_joint_position_.position.a5<<","<<current_joint_position_.position.a6<<","<<current_joint_position_.position.a7<<"]\n"; 
				}
			}
		}
	}
  	myfile.close();

	group.setPoseTarget(transfer_pose_1);
}

bool MoveItNode::plan_and_execute(const geometry_msgs::Pose& msg)
{
	
	// std::stringstream outss;
	// for(auto joint: group.getCurrentJointValues())
	// {
	// 	outss<<joint*180/M_PI<<"  ";
	// }

	// ROS_INFO_STREAM("Planning from start state: ["<<outss.str()<<"]");



	// ROS_INFO_STREAM("Planning for the goal pose: [" << msg.pose.position.x << " " <<
	// 	msg.pose.position.y << " " << msg.pose.position.z << " " << msg.pose.orientation.x << " " <<
	// 	msg.pose.orientation.y << " " <<msg.pose.orientation.z << " " <<msg.pose.orientation.w << "]");
	bool success;
	group.setStartState(*group.getCurrentState());
	group.setPoseTarget(msg);
	//success = (group.plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);

	group.move();
	// if (success)
	// {
	// 	ROS_WARN_STREAM("PLAN_AND_EXECUTE: success to plan");
	// 	//success = group.execute(my_plan);
	// }
	// if(success)
	// {
	// 	ROS_WARN_STREAM("PLAN_AND_EXECUTE: success to execute");
	// }
	return success;
}

bool MoveItNode::plan_and_execute(const geometry_msgs::PoseStamped& msg)
{
	target_pose.pose = msg.pose;
	target_pose.header.frame_id = group.getPlanningFrame();

	// std::stringstream outss;
	// for(auto joint: group.getCurrentJointValues())
	// {
	// 	outss<<joint*180/M_PI<<"  ";
	// }

	// ROS_INFO_STREAM("Planning from start state: ["<<outss.str()<<"]");



	// ROS_INFO_STREAM("Planning for the goal pose: [" << msg.pose.position.x << " " <<
	// 	msg.pose.position.y << " " << msg.pose.position.z << " " << msg.pose.orientation.x << " " <<
	// 	msg.pose.orientation.y << " " <<msg.pose.orientation.z << " " <<msg.pose.orientation.w << "]");

	group.setStartState(*group.getCurrentState());
	group.setPoseTarget(target_pose);
	success = (group.plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);

	group.move();
	// if (success)
	// {
	// 	ROS_WARN_STREAM("PLAN_AND_EXECUTE: success to plan");
	// 	//success = group.execute(my_plan);
	// }
	// if(success)
	// {
	// 	ROS_WARN_STREAM("PLAN_AND_EXECUTE: success to execute");
	// }
	return success;
}



bool MoveItNode::plan_and_execute(std::vector<double> robot_state)
{
	group.setJointValueTarget(robot_state);
	group.move();
	return 1;
}

bool MoveItNode::goto_home()
{
	std::vector<double> command_joint_position = {0, 0, 0, -90 * M_PI/180, 0, 90 * M_PI/180, 0};

	group.setJointValueTarget(command_joint_position);

	success = (bool)group.plan(my_plan);
	if (success)
	{
        group.move();
	}
    
	return success;
}
bool MoveItNode::plan_and_execute_via_waypoints(const geometry_msgs::PoseStamped& target_pose)
{
	geometry_msgs::Pose current_pose = group.getCurrentPose().pose;
	double fraction = plan_and_execute_via_waypoints(current_pose,target_pose.pose,NUM_MIDPTS);
	// if(plan_and_execute_via_waypoints(current_pose,target_pose.pose,5) > 0)
	if(std::abs(fraction-1.0) <  0.01)
	{
		ROS_WARN_STREAM("Successfully completed Cartesian control");
		return true;
	}
	else
	{
		ROS_ERROR_STREAM("FAILED to complete Cartesian control");
		return false;
	}
}
bool MoveItNode::plan_and_execute_via_waypoints(const geometry_msgs::PoseStamped& target_pose, bool keep_orientation)
{
	geometry_msgs::Pose current_pose = group.getCurrentPose().pose;
	auto oriented_target_pose = target_pose;
	oriented_target_pose.pose.orientation = current_pose.orientation;
	double fraction = plan_and_execute_via_waypoints(current_pose,oriented_target_pose.pose,NUM_MIDPTS);
	// if(plan_and_execute_via_waypoints(current_pose,target_pose.pose,5) > 0)
	if(std::abs(fraction-1.0) <  0.01)
	{
		ROS_WARN_STREAM("Successfully completed Cartesian control");
		return true;
	}
	else
	{
		ROS_ERROR_STREAM("FAILED to complete Cartesian control");
		return false;
	}
}
bool MoveItNode::plan_and_execute_via_waypoints(const geometry_msgs::Pose& target_pose)
{
	geometry_msgs::PoseStamped ps;
	ps.pose = target_pose;
	return plan_and_execute_via_waypoints(ps);
}

double MoveItNode::plan_and_execute_via_waypoints(double x, double y, double z, double num_midpts)
{
	geometry_msgs::Pose current_pose = group.getCurrentPose().pose;
	geometry_msgs::Pose target_pose = current_pose;
	target_pose.position.x+=x;
	target_pose.position.y+=y;
	target_pose.position.z+=z;
	return plan_and_execute_via_waypoints(current_pose, target_pose, num_midpts);
}

// [changkyu]
bool MoveItNode::plan_and_execute_U_push(const geometry_msgs::Pose start_p, double x, double y, double z_buf, double num_midpts)
{
    // define U shape pusing poses
    std::vector<geometry_msgs::Pose> waypoints_pre;
    geometry_msgs::Pose pre_p    = start_p;
    geometry_msgs::Pose target_p = start_p;    
    pre_p.position.z += z_buf;
    target_p.position.x += x;
    target_p.position.y += y;
    geometry_msgs::Pose post_p = target_p;    
    post_p.position.z += z_buf;

    std::vector<geometry_msgs::Pose> waypoints_all;
    std::vector<geometry_msgs::Pose> waypoints_local[4];
    geometry_msgs::Pose nxt_p[4] = {pre_p, start_p, target_p, post_p};
    geometry_msgs::Pose cur_p = group.getCurrentPose().pose;
    for( int w=0; w<4; w++ )
    {
        if( quaternion_similarity(cur_p, nxt_p[w])==false )
        {
            cur_p.orientation = nxt_p[w].orientation;
            waypoints_local[w].push_back(cur_p);
        }

        double vec_x = (nxt_p[w].position.x-cur_p.position.x);
        double vec_y = (nxt_p[w].position.y-cur_p.position.y);
        double vec_z = (nxt_p[w].position.z-cur_p.position.z);
        if( std::abs(std::sqrt((vec_x*vec_x)+(vec_y*vec_y)+(vec_z*vec_z))) > 0.001 )
        {
            for (int i=0; i<num_midpts-1; i++)
            {
                cur_p.position.x += vec_x/num_midpts;
                cur_p.position.y += vec_y/num_midpts;
                cur_p.position.z += vec_z/num_midpts;
                waypoints_local[w].push_back(cur_p);
            }
        }
        cur_p = nxt_p[w];
        waypoints_local[w].push_back(cur_p);

        for (int i=0; i<waypoints_local[w].size(); i++)
        {
            waypoints_all.push_back(waypoints_local[w][i]);
        }
    }

    moveit_msgs::RobotTrajectory trajectory_all;
    group.setStartState(*group.getCurrentState());
    double fraction = group.computeCartesianPath(waypoints_all, eef_step, jump_threshold, trajectory_all);
    ROS_ERROR_STREAM("Waypoint fractional completion for all trajectory..."<<fraction);
    if (std::abs(fraction-1.0) <  0.01)
    {
        for( int w=0; w<4; w++ )
        {
            moveit_msgs::RobotTrajectory trajectory;
            group.setStartState(*group.getCurrentState());
            double fraction = group.computeCartesianPath(waypoints_local[w], eef_step, jump_threshold, trajectory);
            ROS_ERROR_STREAM("Waypoint fractional completion for [" << w+1 << "/4" << "]..." <<fraction);
            if (std::abs(fraction-1.0) <  0.01)
            {
                my_plan.trajectory_ = trajectory;
                if(trajectory.joint_trajectory.points.back().time_from_start.toSec() < cartesian_plan_limit)
                {
                    execute_trajectory(my_plan);
                    ros::Duration(1.0).sleep();
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool MoveItNode::quaternion_similarity(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
	double SIM_THRESHOLD = 0.001;
	auto q1 = p1.orientation;
	auto q2 = p2.orientation;

	Eigen::Quaternionf eq1;
	eq1.x()=q1.x;
	eq1.y()=q1.y;
	eq1.z()=q1.z;
	eq1.w()=q1.w;
	Eigen::Quaternionf eq2;
	eq2.x()=q2.x;
	eq2.y()=q2.y;
	eq2.z()=q2.z;
	eq2.w()=q2.w;

	eq1.normalize();
	eq2.normalize();



	// ROS_INFO_STREAM(p1<<" \n ---------------- \n "<<p2);

	// if( 	std::abs(q1.x-q2.x) < SIM_THRESHOLD 
	// 	&&	std::abs(q1.y-q2.y) < SIM_THRESHOLD 
	// 	&&	std::abs(q1.z-q2.z) < SIM_THRESHOLD 
	// 	&&	std::abs(q1.w-q2.w) < SIM_THRESHOLD
	// 	)


	if( std::abs ( std::abs(eq1.dot(eq2)) -1 ) < SIM_THRESHOLD  )
	{
		// ROS_WARN_STREAM("Orientations same.");
		return true;
	}
	else
	{
		// ROS_ERROR_STREAM("Orientations differ.");
		return false;
	}
}


double MoveItNode::plan_and_execute_via_waypoints(const geometry_msgs::Pose& start_p, const geometry_msgs::Pose& target_p, double num_midpts)
{
	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose mid_p, transformed_start_p;

	transformed_start_p = start_p;
	transformed_start_p.orientation = target_p.orientation;




	// waypoints.push_back(start_p);
	bool orientations_same = quaternion_similarity(transformed_start_p, start_p);
	if(!orientations_same)
	{
		waypoints.push_back(transformed_start_p);
	}


	double delx = (start_p.position.x-target_p.position.x);
	double dely = (start_p.position.y-target_p.position.y);
	double delz = (start_p.position.z-target_p.position.z);
	bool positions_same = std::abs(std::sqrt((delx*delx)+(dely*dely)+(delz*delz))) < 0.001;

	if(orientations_same && positions_same)
	{
		ROS_WARN_STREAM("Asked to cartesian control to identical goal point.");
		return 1.0;
	}



	for (int i = 1; i < num_midpts; ++i)
	{
		// mid_p = start_p;
		// mid_p.position.x += (target_p.position.x - start_p.position.x)/num_midpts * i;
		// mid_p.position.y += (target_p.position.y - start_p.position.y)/num_midpts * i;
		// mid_p.position.z += (target_p.position.z - start_p.position.z)/num_midpts * i;
		mid_p = transformed_start_p;
		mid_p.position.x += (target_p.position.x - transformed_start_p.position.x)/num_midpts * i;
		mid_p.position.y += (target_p.position.y - transformed_start_p.position.y)/num_midpts * i;
		mid_p.position.z += (target_p.position.z - transformed_start_p.position.z)/num_midpts * i;
		waypoints.push_back(mid_p);
	}

	waypoints.push_back(target_p);

	moveit_msgs::RobotTrajectory trajectory;

	group.setStartState(*group.getCurrentState());

	double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

	ROS_ERROR_STREAM("Waypoint fractional completion..."<<fraction);
	// if (fraction > 0)
	if (std::abs(fraction-1.0) <  0.01)
	{
		my_plan.trajectory_ = trajectory;
		// ROS_WARN_STREAM("Points in trajectory ||| "<<trajectory.joint_trajectory.points.size());
		// ROS_WARN_STREAM("Final point in trajectory ||| "<<trajectory.joint_trajectory.points.back().time_from_start);
		
		if(trajectory.joint_trajectory.points.back().time_from_start.toSec() < cartesian_plan_limit)
		{
			execute_trajectory(my_plan);
		}
		else
		{
			// fraction = 0;
			// double recursive_fraction;
			// moveit_msgs::RobotTrajectory retry_trajectory;
			// waypoints.clear();
			// // waypoints.push_back(start_p);
			// waypoints.push_back(transformed_start_p);
			// recursive_fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, retry_trajectory);
			// if (std::abs(recursive_fraction-1.0) <  0.01)
			// {
			// 	my_plan.trajectory_ = retry_trajectory;
			// 	ROS_WARN_STREAM("RETRYING Points in trajectory ||| "<<retry_trajectory.joint_trajectory.points.size());
			// 	ROS_WARN_STREAM("RETRYING Final point in trajectory ||| "<<retry_trajectory.joint_trajectory.points.back().time_from_start);
				
			// 	if(retry_trajectory.joint_trajectory.points.back().time_from_start.toSec() < cartesian_plan_limit)
			// 	{
			// 		execute_trajectory(my_plan);
			// 	}

			// }
		}

	}


	return fraction;

}

double MoveItNode::plan_and_execute_via_waypoints(const std::vector<geometry_msgs::Pose> &waypoints)
{
    moveit_msgs::RobotTrajectory trajectory;
    group.setStartState(*group.getCurrentState());
    double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_ERROR_STREAM("Waypoint fractional completion..."<<fraction);
    if (std::abs(fraction-1.0) <  0.01)
    {
        my_plan.trajectory_ = trajectory;
        if(trajectory.joint_trajectory.points.back().time_from_start.toSec() < cartesian_plan_limit)
        {
            execute_trajectory(my_plan);
        }
        else
        {
            ROS_ERROR_STREAM("Exceed Cartesian Plan Limit: " << trajectory.joint_trajectory.points.back().time_from_start.toSec() << " < " << cartesian_plan_limit);
        }
    }
    return fraction;
}

void MoveItNode::addGripperObject()
{
	moveit_msgs::AttachedCollisionObject gripper_box;
	gripper_box.link_name = "iiwa_link_ee";
	gripper_box.object.header.frame_id = "iiwa_link_ee";
	gripper_box.object.id = "gripper";

	geometry_msgs::Pose gripper_pose;
	gripper_pose.position.x = 0.0;
	gripper_pose.position.y = 0.0;
	gripper_pose.position.z = 0.125 + 0.01 + 0.05;
	gripper_pose.orientation.w = 0.38268343;
	gripper_pose.orientation.z = 0.92387953;

	shape_msgs::SolidPrimitive primitive_;
	primitive_.type = primitive_.BOX;
	primitive_.dimensions.resize(3);
	primitive_.dimensions[0] = 0.16;
	primitive_.dimensions[1] = 0.16;
	primitive_.dimensions[2] = 0.27 - 0.10;

	gripper_box.object.primitives.push_back(primitive_);
	gripper_box.object.primitive_poses.push_back(gripper_pose);

	gripper_box.object.operation = gripper_box.object.ADD;

	ROS_INFO("Adding the gripper box into the world...");
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.world.collision_objects.push_back(gripper_box.object);
	planning_scene.is_diff = true;
	planning_scene_diff_publisher.publish(planning_scene);
	ros::Duration(2.0).sleep();

	moveit_msgs::CollisionObject remove_object;
	remove_object.id = "gripper";
	remove_object.header.frame_id = "iiwa_link_ee";
	remove_object.operation = remove_object.REMOVE;

	ROS_INFO("Attaching the gripper to the end effector and removing it from the world.");
	planning_scene.world.collision_objects.clear();
	planning_scene.world.collision_objects.push_back(remove_object);
	planning_scene.robot_state.attached_collision_objects.push_back(gripper_box);
	planning_scene_diff_publisher.publish(planning_scene);

	ros::Duration(2.0).sleep();
}

void MoveItNode::printJointPosition(iiwa_msgs::JointPosition msg){
	ROS_INFO_STREAM("JointPosition:["<<msg.position.a1<<","<<msg.position.a2<<","<<msg.position.a3<<","<<msg.position.a4<<","<<msg.position.a5<<","<<msg.position.a6<<","<<msg.position.a7<<"]");
}