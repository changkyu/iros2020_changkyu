#include <utils.hpp>

namespace utilities
{
	void sleepForMotion(iiwa_ros::iiwaRos& iiwa, const double maxSleepTime)
	{
		double ttd = iiwa.getTimeToDestinationService().getTimeToDestination();
		ros::Time start_wait = ros::Time::now();

		while (ttd < 0.0 && (ros::Time::now() - start_wait) < ros::Duration(maxSleepTime))
		{
			ROS_INFO_STREAM("Raw ttd received: " << ttd);
			ros::Duration(0.05).sleep();
			ttd = iiwa.getTimeToDestinationService().getTimeToDestination();
		}
		if (ttd > 0.0)
		{
			ROS_INFO_STREAM("Sleeping for " << ttd << " seconds.");
			// ros::Duration(ttd + 0.2).sleep();
			ros::Duration(ttd).sleep();
		}
	}



	void goToHomePosition(iiwa_ros::iiwaRos& my_iiwa)
	{
		// The current home joint position is encoded (in degrees) as:
		// [0, 0, 0, -30, 0, 90, 0]
		iiwa_msgs::JointPosition command_joint_position;

		command_joint_position.position.a4 = -30 * M_PI / 180;
		command_joint_position.position.a6 =  90 * M_PI / 180;
		my_iiwa.setJointPosition(command_joint_position);

		utilities::sleepForMotion(my_iiwa,2.0);
	}

	std::string filename;
	std::ofstream log_file;
	void writeToLog(std::string str){
		//If the filename is empty that means none of the planners had the parameter stat_file
    #ifdef COLLECT_DATA   
        if(filename.empty())
        {
        	if(ros::param::has("/jdx/log_filename"))
      		{
      			ros::param::get("/jdx/log_filename", filename);
      		}
	    	else
	    	{
	      		ROS_WARN_STREAM("Could not find the log_filename...");
	      		exit(1);
	    	}
        }
        time_t rawtime;
  		struct tm * timeinfo;
  		char buffer[80];
  		time (&rawtime);
  		timeinfo = localtime(&rawtime);

  		strftime(buffer,sizeof(buffer),"%d_%m_%Y_%H_%M_%S",timeinfo);
  		std::string time_string(buffer);        
        if(!filename.empty())
        {
	        log_file.open (filename.c_str(), std::ios::out | std::ios::app);
	        log_file <<time_string + ":  " +  str + "\n";
	        log_file.close();
        }
        else
        {
            //The file where to write the statistics is unknown
            ROS_ERROR_STREAM("No stat file defined in ");
        }
    #endif
	}

	std::string wait_for_response(std::string prompt)
	{
		ROS_WARN_STREAM(prompt);
		std::string x;
		std::cin>>x;
		return x;
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
}
