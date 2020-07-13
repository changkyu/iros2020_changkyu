/*
File: JDXDemo.cpp

Authors: Aravind Sivaramakrishnan

Description:

Comments/TODO:
-
-
*/

#include <JDXDemo.hpp>
#include <algorithm>
#include <functional>
#include <ctime>
#include <iomanip>

#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <chrono>

#include "rl_msgs/organizing_pusher_jdxdemo_srv.h"

#include <cstdlib>

//The number of picks after which the experiment is declared a success and switches bins
#define MAX_OBJECTS_TO_PICK 9
//Time limit beyond which keyboard input is required to continue the experiment
#define MAX_TIME_LIMIT 600
//Set this flag to collect data. The folder locations and parameters must be set appropriately
// #define COLLECT_DATA

std::string experiment_folder_name = "";

Demo::Demo()
{
	//Pose of the target bin
	place_pose_1.pose.position.x = 0.39;
	place_pose_1.pose.position.y = -0.34;
	place_pose_1.pose.position.z = 0.12;
	place_pose_1.pose.orientation.x = -0.38268343;
	place_pose_1.pose.orientation.y = 0.92387953;
	place_pose_1.pose.orientation.z = 0.0;
	place_pose_1.pose.orientation.w = 0.0;

	//Pose of the source bin
	place_pose_2.pose.position.x = 0.39;
	place_pose_2.pose.position.y = 0.34;
	place_pose_2.pose.position.z = 0.12;
	place_pose_2.pose.orientation.x = -0.38268343;
	place_pose_2.pose.orientation.y = 0.92387953;
	place_pose_2.pose.orientation.z = 0.0;
	place_pose_2.pose.orientation.w = 0.0;
	
	//These two variables keep track of which bin is being used as the source and target, and will toggle the appropriate behavior down the pipeline
	source_bin = "bin_1";
	target_bin = "bin_2";



	comm = new perception_communication();
	task_planner.set_communication_link(comm);
	task_planner.set_placement_module(&placement_mod);
	marker_timer = nh.createTimer(ros::Duration(1), publish_arrow);
	MAX_TIME = MAX_TIME_LIMIT;

	//YCB
	// original_object_list = {
	// 	 "004_sugar_box",
	// 	 "005_tomato_soup_can",
	// 	 "006_mustard_bottle",
	// 	 "008_pudding_box",
	// 	 "009_gelatin_box",
	// 	 "010_potted_meat_can",
	// 	 "024_bowl",
	// 	 // "011_banana",
	// 	 "025_mug",
	// 	 "002_master_chef_can"
	// };

	//INSTANCE
	original_object_list = {
		"dove",
		 "toothpaste"
	};
	// original_object_list = {
	// 	 "dove"
	// };
	// original_object_list = {
	// 	 "toothpaste"
	// };

	//Currently the implementation can only handle one object placement at once, so this needs to be toggled for the object of interest
	//placement_mod.setup("dove");
	obj_num_per_layer = placement_mod.setup("dove");
	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();

	//set default location for dumping data
	experiment_folder_name = "~/";
}

void Demo::reset_object_attempts()
{
	for(auto obj: original_object_list)
	{
		object_attempts_map[obj] = 0;
	}
}

void Demo::get_objects_to_sense(std::vector<std::string>& object_list, int MAX_ATTEMPTS)
{
	object_list.clear();
	ROS_WARN_STREAM("####################################");
	ROS_WARN_STREAM("####################################");
	ROS_ERROR_STREAM("Objects to SENSE : ");
	for(auto obj: original_object_list)
	{
		if(object_attempts_map[obj]<MAX_ATTEMPTS)
		{
			object_list.push_back(obj);
			ROS_ERROR_STREAM("  "<<obj);
		}
	}
	ROS_ERROR_STREAM("Objects to SENSE : ");
	ROS_WARN_STREAM("####################################");
	ROS_WARN_STREAM("####################################");
}

int Demo::get_gripper_control(double width)
{
	double gripper_control = (int)( 80 + (0.5-width) * 100 );
	if(gripper_control>90)
		gripper_control = 90;
	if(gripper_control<40)
		gripper_control = 40;
	return gripper_control;
}

bool Demo::run_experiment()
{
	bool VERIFY_EMPTY_BIN = true;
	int current_source_bin = 1;
	while (ros::ok())
	{
		ROS_INFO_STREAM("Current state at Demo: " << get_state(current_state));

		if (current_state == START)
		{
			utilities::writeToLog("Starting Experiment");

			char wait_for_input;
			std::cout<<"\nPress n to end...\n";
			std::cin>>wait_for_input;
			if (wait_for_input == 'n')
			{
				return true; 
			}

			// task_planner.test_joint_velocity();

			// wait_for_input;
			// std::cout<<"\nPress n to end...\n";
			// std::cin>>wait_for_input;
			// if (wait_for_input == 'n')
			// {
			// 	return true; 
			// }
			//task_planner.executeTask("PREPLAN");
			

			//What is this? -Rahul
			task_planner.validateCalibration();



			std::string tmp_bin;
			tmp_bin = target_bin;
			target_bin = source_bin;
			source_bin = tmp_bin;

			reset_object_attempts();
			VERIFY_EMPTY_BIN = true;


			//Specify bin and object list to sensing
			get_objects_to_sense(object_list);
			if(source_bin == "bin_1")
				current_source_bin = 1;
			else if(source_bin == "bin_2")
				current_source_bin = 2;

			comm->set_sensing(current_source_bin, object_list);
			comm->start_sensing();

			execution_result = task_planner.executeTask("HOME");

			current_state = execution_result ? SENSE : FINISH;
			// std::cout<<"\nPress n to end...\n";
			// std::cin>>wait_for_input;
			// if (wait_for_input == 'n')
			// {
			// 	return true; 
			// }
			//##########################################
			//##########################################
			// Waypoints Test
			// while(utilities::wait_for_response("Enter q to quit...") != "q")
			// {
			// 	std::string x,y,z,m;
			// 	x = utilities::wait_for_response("X offset?");
			// 	y = utilities::wait_for_response("Y offset?");
			// 	z = utilities::wait_for_response("Z offset?");
			// 	m = utilities::wait_for_response("Interpolation points?");
			// 	// task_planner.get_planning_node()->plan_and_execute_via_waypoints(0,0,-10);
			// 	task_planner.get_planning_node()->plan_and_execute_via_waypoints(std::stod(x),std::stod(y),std::stod(z),std::stoi(m));
			// }
			//##########################################
			//##########################################



			
			//ros::Duration(10.0).sleep();
			 // ROS_WARN_STREAM("FINISH PREPLAN");
			// std::cout<<"\nAnother Press n to end...\n";
			// std::cin>>wait_for_input;
			// if (wait_for_input == 'n')
			// {
			// 	return true; 
			// }
			//task_planner.executeTask("TEST_TRANSFER");

			//current_state = FINISH;
		}
		else if (current_state == SENSE)
		{	
		
			obj_srv.request.normal.x = 0;
			obj_srv.request.normal.y = 0;
			obj_srv.request.normal.z = -1;
			
			std::vector<response_t> sensing_result;
			#ifdef USE_GAZEBO
			if(OBJECT_SOURCE == 1){ 
				// object pose is from pose estimation from real world
				sensing_result = comm->get_sensing();
			}else if(OBJECT_SOURCE == 0){
				// object pose is self-generated
				std::string object_name("006_mustard_bottle");
				geometry_msgs::Pose object_pose;
				object_pose.position.x = 0.41;
				object_pose.position.y = 0.35;
				object_pose.position.z = 0.895;
				object_pose.orientation.x =  0.6123724;
				object_pose.orientation.y =  0.6123724;
				object_pose.orientation.z =  0.3535534;
				object_pose.orientation.w =  0.3535534;
				spawnObject(nh, "/home/cm1074/.gazebo/models/"+object_name+"/model-1_4.sdf", 
    					object_name, object_pose);
				object_pose.position.z -= 1;
				sensing_result = comm->get_sensing_from_object_pose(object_name, object_pose);
			}

			#else
				ROS_WARN_STREAM("Getting sensing information....");
				sensing_result = comm->get_sensing();
			#endif
			// //## TESTING
			// response_t result;
			// result.end_effector_pose.x = 0.39;
			// result.end_effector_pose.z = -0.15;
			// result.end_effector_pose.qx = 1;
			// result.end_effector_pose.qy = 0;
			// result.end_effector_pose.qz = 0;
			// result.end_effector_pose.qw = 0;
			
			// if(source_bin == "bin_1")
			// { 
			// 	result.end_effector_pose.y = -0.34;
			// }
			// else if(source_bin == "bin_2")
			// {
			// 	result.end_effector_pose.y = 0.34;
			// }
			// ROS_ERROR_STREAM("Will attempt to simulate a pick at "<<result.end_effector_pose.print());
			// sensing_result.push_back(result);
			// //## TESTING
			object_poses.clear();
			object_detections.clear();
			object_labels.clear();

			if(!sensing_result.empty())
			{
				if(sensing_result.size()>1)
				{
					ROS_ERROR_STREAM("Should be getting only one surface right now...");
					exit(1);
				}

				geometry_msgs::Pose ee_pose;
				ee_pose.position.x = sensing_result[0].end_effector_pose.x;
				ee_pose.position.y = sensing_result[0].end_effector_pose.y;
				ee_pose.position.z = sensing_result[0].end_effector_pose.z;
				ee_pose.orientation.x = sensing_result[0].end_effector_pose.qx;
				ee_pose.orientation.y = sensing_result[0].end_effector_pose.qy;
				ee_pose.orientation.z = sensing_result[0].end_effector_pose.qz;
				ee_pose.orientation.w = sensing_result[0].end_effector_pose.qw;
				object_poses.push_back(ee_pose);

				task_planner.get_grasp_node()->set_grasp_opening(get_gripper_control(sensing_result[0].object_width));

				object_attempts_map[sensing_result[0].object_name]++;

				current_state = PICK_AND_PLACE;
				ROS_WARN_STREAM("Finished Automaton stage SENSE");
			}
			else
			{
				ROS_ERROR_STREAM("NO OBJECTS SENSED.");
				//If this flag is true that means for this bin, 
				//we have not checked with all the objects to verify whether something is left
				if(VERIFY_EMPTY_BIN)
				{
					ROS_ERROR_STREAM("Verifying the Bin Contents.");
					//Test for all the objects just to be sure
					reset_object_attempts();
					get_objects_to_sense(object_list);
					comm->set_sensing(current_source_bin, object_list);
					comm->start_sensing();
					current_state = SENSE;
					VERIFY_EMPTY_BIN = false;
				}
				else
				{
					ROS_ERROR_STREAM("Finishing the loop.");
					//We have checked with all the objects and found the bin to be empty
					current_state = FINISH;
				}
			}
		}
		else if (current_state == PICK_AND_PLACE)
		{
			execution_result = false;
			if (object_poses.size() != 0 )
			{
				VERIFY_EMPTY_BIN = true;
				ROS_WARN_STREAM("Beginning Automaton stage PaP");
				get_objects_to_sense(object_list);
				comm->set_sensing(current_source_bin, object_list);
				ROS_WARN_STREAM("Set specification for sensing.");

				// if(object_poses.at(0).position.z>-999 )//This condition deals with lack of graspable points
				if(object_poses.at(0).position.z < (-0.175) )//This condition deals with lack of graspable points, or too low points
				{
					target_pose.pose = object_poses.at(0);
					object_poses.erase(object_poses.begin());
					#ifndef USE_GAZEBO
					ros::ServiceClient grasp_planning_client = nh.serviceClient<grasping::GraspPlanning>("/CollisionGraspPlanning");
					grasping::GraspPlanning srv;
					if(target_bin == "bin_2")
						srv.request.binId = 1;
					else if(target_bin == "bin_1")
						srv.request.binId = 2;
					std::cout<<target_pose.pose.position.x<<","<<target_pose.pose.position.y<<std::endl;
					srv.request.target_pose = target_pose.pose;
					int gripper_collision_num = 0;
					if (grasp_planning_client.call(srv))
				  	{
				    	gripper_collision_num = srv.response.collision_num;
				    	if(gripper_collision_num > 2700){
				    		ROS_WARN_STREAM("Current grasp is in collision, re-sense");
				    		current_state = SENSE;
				    		continue;
				    	}	
				  	}
				  	else
				  	{
				    	ROS_ERROR("Failed to call service check grasp collision");
				    	return 1;
				  	}	
				  	#endif
					if(target_bin == "bin_2")
					{ 
						execution_result = task_planner.executeTask("PICK_AND_PLACE",target_pose,place_pose_2,0.5, source_bin, target_bin);
						auto gfm = task_planner.get_grasp_node()->grasping_failure_mode;
						#ifdef REFLEX
						//#### ROBOTIQ
						if(!execution_result &&  ( gfm == GraspNode::PREGRASP_MP || gfm == GraspNode::GRASP_MP ) )
							execution_result = task_planner.executeTask("PICK_AND_PLACE", task_planner.get_orthogonal_pose(target_pose),place_pose_2,0.5, source_bin, target_bin);
						#endif
					}
					else if(target_bin == "bin_1")
					{
						execution_result = task_planner.executeTask("PICK_AND_PLACE",target_pose,place_pose_1,0.5, source_bin, target_bin);					
						auto gfm = task_planner.get_grasp_node()->grasping_failure_mode;
						#ifdef REFLEX
						//#### ROBOTIQ
						if(!execution_result &&  ( gfm == GraspNode::PREGRASP_MP || gfm == GraspNode::GRASP_MP ) ) 
							execution_result = task_planner.executeTask("PICK_AND_PLACE", task_planner.get_orthogonal_pose(target_pose),place_pose_1,0.5, source_bin, target_bin);					
						#endif
					}
				}

				if(!execution_result)
				{
					comm->start_sensing();
				}
			}

			current_state = SENSE;
		}
		else if (current_state == FINISH)
		{
			current_state = START;
		}
		loop_rate_->sleep();
	}
}


///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////




void Demo::savePointCloud(std::string bin_name, int count)
{
#ifdef COLLECT_DATA
	ros::NodeHandle nh;
	std::string color_image_topic_param,depth_image_topic_param,color_image_topic,depth_image_topic;
	sensor_msgs::Image::ConstPtr msg_color, msg_depth;

	color_image_topic_param = "/bins/" + bin_name +"/color_image_topic";
	depth_image_topic_param = "/bins/" + bin_name +"/depth_image_topic";
	nh.getParam(color_image_topic_param, color_image_topic);
	nh.getParam(depth_image_topic_param, depth_image_topic);

	std::cout<<"\nThe image topic is: "<<color_image_topic_param;
	std::cout<<"\nThe depth topic is: "<<depth_image_topic_param;

	std::cout << "Waiting for rgb image on topic: " << color_image_topic << std::endl;
	msg_color = ros::topic::waitForMessage<sensor_msgs::Image>(color_image_topic, nh);

	std::cout << "Waiting for depth image on topic: " << depth_image_topic << std::endl;
    msg_depth = ros::topic::waitForMessage<sensor_msgs::Image>(depth_image_topic, nh);

    cv_bridge::CvImagePtr cv_ptr_color;
    cv_bridge::CvImagePtr cv_ptr_depth;
    try {
       cv_ptr_color = cv_bridge::toCvCopy(*msg_color, sensor_msgs::image_encodings::BGR8);
       cv_ptr_depth = cv_bridge::toCvCopy(*msg_depth, (*msg_depth).encoding);
       cv::imwrite(experiment_folder_name + bin_name+"-frame-"+std::to_string(count)+".color.png", cv_ptr_color->image);
       cv::imwrite(experiment_folder_name + bin_name+"-frame-"+std::to_string(count)+".depth.png", cv_ptr_depth->image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());\
        exit(-1);
    }

    std::cout<<"\nJust wrote the images for the "<<bin_name;
#endif

}

void Demo::fix_using_push_and_pull(){
	ros::NodeHandle nh;
	utilities::writeToLog("Calling Push Adjustment - final");
	task_planner.executeTask("HOME");
	ROS_INFO_STREAM("Hi, Changkyu");
	ros::ServiceClient clt_pusher
     = nh.serviceClient<rl_msgs::organizing_pusher_jdxdemo_srv>(
       "/iiwa/changkyu/organizer_jdxdemo/pusher");
    rl_msgs::organizing_pusher_jdxdemo_srv::Request req;
    rl_msgs::organizing_pusher_jdxdemo_srv::Response res;
    req.camera_name = "camera2";
    req.param="thresh=0.006,iter_max=20";    
    clt_pusher.call(req, res);    
    ROS_INFO_STREAM("Bye Changkyu");
    utilities::writeToLog("Done with Push Adjustment - final");
    task_planner.executeTask("HOME");
}

bool Demo::run_experiment_grid()
{
	bool VERIFY_EMPTY_BIN = true;
	int current_source_bin = 1;
	// clock_t startTime = clock(); //Start timer
	auto start_time = std::chrono::high_resolution_clock::now();
 	double seconds_passed;
	int objects_to_pick = 0;
	while (ros::ok())
	{
		ROS_INFO_STREAM("Current state at Demo: " << get_state(current_state));

		auto current_time = std::chrono::high_resolution_clock::now();
		seconds_passed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();


		if(seconds_passed>MAX_TIME)
		{
			std::cout<<"\n Reached the end of "<<seconds_passed<<"/"<<MAX_TIME<<"\n.... Enter c to continue... any other key to try for 100 more seconds";
			std::string input;
			std::cin>>input;

			if(input=="c")
			{
				utilities::writeToLog("Timed out at "+std::to_string(seconds_passed)+"s");
				savePointCloud("bin_1");
				return true;
			}
			else
			{
				MAX_TIME+=100;
			}
			
		}

		if (current_state == START)
		{
			
			char wait_for_input;


			std::string tmp_bin;
			tmp_bin = target_bin;
			target_bin = source_bin;
			source_bin = tmp_bin;

			savePointCloud(source_bin);
			

			std::cout<<"\nPress n to end...\n";
			std::cin>>wait_for_input;
			if (wait_for_input == 'n')
			{
				return true; 
			}

			utilities::writeToLog("\nStarting Experiment");
			task_planner.get_planning_node()->deleteBin();
			reset_object_attempts();
			VERIFY_EMPTY_BIN = true;


			//Specify bin and object list to sensing
			get_objects_to_sense(object_list);
			if(source_bin == "bin_1")
				current_source_bin = 1;
			else if(source_bin == "bin_2")
				current_source_bin = 2;

			comm->set_sensing(current_source_bin, object_list);
			comm->start_sensing();

			execution_result = task_planner.executeTask("HOME");

			current_state = execution_result ? SENSE : FINISH;
		}
		else if (current_state == SENSE)
		{	
			utilities::writeToLog("SENSE Automaton state.");
			obj_srv.request.normal.x = 0;
			obj_srv.request.normal.y = 0;
			obj_srv.request.normal.z = -1;
			
			std::vector<response_t> sensing_result;
			
			ROS_WARN_STREAM("Getting sensing information....");
			sensing_result = comm->get_sensing();

			object_poses.clear();
			object_detections.clear();
			object_labels.clear();
			if(!sensing_result.empty())
			{
				if(sensing_result.size()>1)
				{
					ROS_ERROR_STREAM("Should be getting only one surface right now...");
					exit(1);
				}

				geometry_msgs::Pose ee_pose;
				ee_pose.position.x = sensing_result[0].end_effector_pose.x;
				ee_pose.position.y = sensing_result[0].end_effector_pose.y;
				ee_pose.position.z = sensing_result[0].end_effector_pose.z;
				ee_pose.orientation.x = sensing_result[0].end_effector_pose.qx;
				ee_pose.orientation.y = sensing_result[0].end_effector_pose.qy;
				ee_pose.orientation.z = sensing_result[0].end_effector_pose.qz;
				ee_pose.orientation.w = sensing_result[0].end_effector_pose.qw;
				object_poses.push_back(ee_pose);


				geometry_msgs::Pose obj_detection;
				obj_detection.position.x = sensing_result[0].detection.x;
				obj_detection.position.y = sensing_result[0].detection.y;
				obj_detection.position.z = sensing_result[0].detection.z;
				obj_detection.orientation.x = sensing_result[0].detection.qx;
				obj_detection.orientation.y = sensing_result[0].detection.qy;
				obj_detection.orientation.z = sensing_result[0].detection.qz;
				obj_detection.orientation.w = sensing_result[0].detection.qw;
				object_detections.push_back(obj_detection);

				object_labels.push_back(sensing_result[0].object_name);

				task_planner.get_grasp_node()->set_grasp_opening(get_gripper_control(sensing_result[0].object_width));

				object_attempts_map[sensing_result[0].object_name]++;

				REGRASP = sensing_result[0].regrasp;

				current_state = PICK_AND_PLACE;
				ROS_WARN_STREAM("Finished Automaton stage SENSE");
			}
			else
			{
				ROS_ERROR_STREAM("NO OBJECTS SENSED.");
				//If this flag is true that means for this bin, 
				//we have not checked with all the objects to verify whether something is left
				if(VERIFY_EMPTY_BIN)
				{
					ROS_ERROR_STREAM("Verifying the Bin Contents.");
					//Test for all the objects just to be sure
					reset_object_attempts();
					get_objects_to_sense(object_list);
					comm->set_sensing(current_source_bin, object_list);
					comm->start_sensing();
					current_state = SENSE;
					VERIFY_EMPTY_BIN = false;
				}
				else
				{
					ROS_ERROR_STREAM("Finishing the loop? Enter c to keep sensing...");
					//We have checked with all the objects and found the bin to be empty
					std::string input;
					std::cin>>input;
					utilities::writeToLog("Sensed no objects");
					if(input == "c")
					{
						current_state = SENSE;
						ROS_ERROR_STREAM("Verifying the Bin Contents.");
						//Test for all the objects just to be sure
						reset_object_attempts();
						get_objects_to_sense(object_list);
						comm->set_sensing(current_source_bin, object_list);
						comm->start_sensing();
						VERIFY_EMPTY_BIN = true;
					}
					else
					{
						current_state = FINISH;
					}

				}
			}
		}
		else if (current_state == PICK_AND_PLACE)
		{
			ROS_ERROR_STREAM("AUTOMATON PICK AND DROP");
			execution_result = false;
			if (object_poses.size() != 0 )
			{
				VERIFY_EMPTY_BIN = true;
				ROS_WARN_STREAM("Beginning Automaton stage PaP");
				get_objects_to_sense(object_list);
				comm->set_sensing(current_source_bin, object_list);
				ROS_WARN_STREAM("Set specification for sensing.");

				// if(object_poses[0].position.z>-999 )//This condition deals with lack of graspable points
				if(object_poses.at(0).position.z > (-0.175) )
				{
					target_pose.pose = object_poses.at(0);


					//############################
					//############################
					//############################
					//Calibration offset

					// target_pose.pose.position.x+=0.012;
					// target_pose.pose.position.y+=0.012;
					target_pose.pose.position.x+=(0.0106-0.005-0.0025);
					target_pose.pose.position.y+=(0.014+0.0086+0.0043);

					//############################
					//############################
					//############################


					object_poses.erase(object_poses.begin());
					ros::ServiceClient grasp_planning_client = nh.serviceClient<grasping::GraspPlanning>("/CollisionGraspPlanning");
					grasping::GraspPlanning srv;
					if(target_bin == "bin_2")
						srv.request.binId = 1;
					else if(target_bin == "bin_1")
						srv.request.binId = 2;
					std::cout<<target_pose.pose.position.x<<","<<target_pose.pose.position.y<<std::endl;
					srv.request.target_pose = target_pose.pose;
					int gripper_collision_num = 0;
					if (grasp_planning_client.call(srv))
				  	{
				    	gripper_collision_num = srv.response.collision_num;
				    	if(gripper_collision_num > 2700){
				    		ROS_WARN_STREAM("Current grasp is in collision, re-sense");
				    		current_state = SENSE;
				    		continue;
				    	}	
				  	}
				  	else
				  	{
				    	ROS_ERROR("Failed to call service check grasp collision");
				    	return 1;
				  	}	

					ROS_ERROR_STREAM("Get Target Pose");
				  	auto drop_pair = placement_mod.get_target_pose(object_labels[0],object_detections[0],target_pose.pose,target_bin);
				  	geometry_msgs::PoseStamped drop_ps;
				  	drop_ps.pose = drop_pair.first;
				  	geometry_msgs::Pose target_obj_pose;
				  	target_obj_pose = drop_pair.second;
					ROS_ERROR_STREAM("Call Pick and Drop");
					//execution_result = task_planner.pick_and_drop_with_tightenv(target_pose, drop_ps, object_detections[0], target_obj_pose,0.5, source_bin, target_bin, object_labels[0], REGRASP);				
					
					execution_result = task_planner.pick_and_drop(target_pose, drop_ps, object_detections[0], target_obj_pose,0.5, source_bin, target_bin, object_labels[0], REGRASP);				
					//execution_result = task_planner.pick_and_drop_without_push(target_pose, drop_ps, object_detections[0], target_obj_pose,0.5, source_bin, target_bin, object_labels[0], REGRASP);				
					// execution_result = task_planner.pick_and_drop_without_topple(target_pose, drop_ps, object_detections[0], target_obj_pose,0.5, source_bin, target_bin, object_labels[0], REGRASP);				
					// execution_result = task_planner.pick_and_drop_without_pose_estimation(target_pose, drop_ps, object_detections[0], target_obj_pose,0.5, source_bin, target_bin, object_labels[0], REGRASP);				
					
				}
				else
				{
					utilities::writeToLog("PICK AND PLACE: Failed Z filter");
				}

				if(!execution_result)
				{
					comm->start_sensing();
				}
				else
				{
					//Take the explicit decision to increment the internal placement counter
					++objects_to_pick;
					std::string input;
					//std::cin>>input;
					ROS_WARN_STREAM(objects_to_pick<<",  obj_num_per_layer:"<<obj_num_per_layer);
					if(objects_to_pick ==  obj_num_per_layer){
						
						fix_using_push_and_pull();
						std::cin>>input;
					}
					placement_mod.increment_counter_on_success(target_bin);
				}
			}
			if(objects_to_pick<MAX_OBJECTS_TO_PICK && task_planner.regrasp_failure < 5)
			{
				current_state = SENSE;
			}
			else
			{
				current_state = FINISH;
			}		
		}
		else if (current_state == FINISH)
		{
			current_state = START;

			
			auto current_time = std::chrono::high_resolution_clock::now();
			seconds_passed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

			utilities::writeToLog("Finished Experiment at "+std::to_string(seconds_passed)+"s");

		}
		loop_rate_->sleep();
	}
}




std::string Demo::get_state(State current_state)
{
	switch(current_state)
	{
		case START: return "START";
		case SENSE: return "SENSE";
		case PICK_AND_PLACE: return "PICK_AND_PLACE";
		case FINISH: return "FINISH";
		default: return "Bad state!";
	}
}

void Demo::update_object_poses()
{
	object_poses.clear();
	object_detections.clear();
	object_labels.clear();
	ROS_WARN_STREAM("Detected "<<obj_srv.response.object_messages.size()<<" objects");
	for (std::vector<task_planning::ObjectPose>::const_iterator it = obj_srv.response.object_messages.begin(); it != obj_srv.response.object_messages.end(); ++it)
	{
		ROS_INFO_STREAM("Updating the pose for object " << it->label);
		object_poses.push_back(it->pose);
	}

	std::sort(object_poses.begin(), object_poses.end(), [](const geometry_msgs::Pose& left, const geometry_msgs::Pose& right){return left.position.z > right.position.z;});

	ROS_WARN_STREAM("Debug sorting...");
	for(auto obj : object_poses)
	{
		ROS_WARN_STREAM("Obj: "<<obj.position.x<<" "<<obj.position.y<<" "<<obj.position.z);
	}
	ROS_WARN_STREAM("Debugged sorting...");
}

int main(int argc, char**argv)
{
	ros::init(argc,argv,"JDX_Demo");

	ros::AsyncSpinner spinner(4);
	spinner.start();

	Demo demo;

//###################################################################
//###################################################################
//#######################Start Video Recording#######################
//###################################################################
//###################################################################
//###################################################################
#ifdef COLLECT_DATA
	std::string root_folder_name = "/home/cm1074/kuka_ws/icra_2018";
	std::string command = "python "+root_folder_name+"/../log_experiments.py";
	std::system(command.c_str());

	std::string experiment_folder_name_file = root_folder_name+"/../current_folder.txt";	
	ifstream in;    // Create an input file stream.
    in.open(experiment_folder_name_file);  // Use it to read from a file named data.txt.
    in >> experiment_folder_name;    
	experiment_folder_name+="/";
    std::cout<<"The currently read output folder is :"<<experiment_folder_name;
    ros::param::set("/jdx/log_filename", experiment_folder_name+"/log.txt");

	time_t rawtime;
  	struct tm * timeinfo;
  	char buffer[80];

  	time (&rawtime);
  	timeinfo = localtime(&rawtime);

  	strftime(buffer,sizeof(buffer),"%d_%m_%Y_%H_%M_%S",timeinfo);
 	std::string filename(buffer);
	command = "guvcview -r none -m none -g none -u raw -j "+experiment_folder_name + filename +".mp4 -y 10000 -x 1920x1080 -a none &";
	system(command.c_str());
#endif


//###################################################################
//###################################################################
//#######################Start Experiment############################
//###################################################################
//###################################################################
//###################################################################
	//bool success = demo.run_experiment();
	//Function that runs the high level automaton loop
	bool success = demo.run_experiment_grid();



//###################################################################
//###################################################################
//############################End Experiment#########################
//###################################################################
//###################################################################
//###################################################################
#ifdef COLLECT_DATA
	system("killall -s USR1 guvcview && sleep 2 && killall -s INT guvcview");
#endif



//###################################################################
//###################################################################
//#####################Final Push Adjustment#########################
//###################################################################
//###################################################################
//###################################################################

#ifdef USE_PUSHING
	//Add bin 
	demo.task_planner.get_planning_node()->addBin();

#ifdef COLLECT_DATA
	//Start Video
	time (&rawtime);
   	timeinfo = localtime(&rawtime);

   	strftime(buffer,sizeof(buffer),"%d_%m_%Y_%H_%M_%S",timeinfo);
  	std::string filename2(buffer);
	command = "guvcview -r none -m none -g none -u raw -j "+experiment_folder_name + filename2 +".mp4 -y 10000 -x 1920x1080 -a none &";
	system(command.c_str());
#endif
	// //Call pushing	

	ros::NodeHandle nh;
	utilities::writeToLog("Calling Push Adjustment - final");
	demo.task_planner.executeTask("HOME");
	ROS_INFO_STREAM("Hi, Changkyu");
	ros::ServiceClient clt_pusher
     = nh.serviceClient<rl_msgs::organizing_pusher_jdxdemo_srv>(
       "/iiwa/changkyu/organizer_jdxdemo/pusher");
    rl_msgs::organizing_pusher_jdxdemo_srv::Request req;
    rl_msgs::organizing_pusher_jdxdemo_srv::Response res;
    req.camera_name = "camera2";
    req.param="thresh=0.006,iter_max=20";    
    clt_pusher.call(req, res);    
    ROS_INFO_STREAM("Bye Changkyu");
    utilities::writeToLog("Done with Push Adjustment - final");

#ifdef COLLECT_DATA
    system("killall -s USR1 guvcview && sleep 2 && killall -s INT guvcview");
#endif
#endif
	demo.task_planner.executeTask("HOME");
	//Save images
	demo.savePointCloud("bin_1", 1);

	return 0;
}
