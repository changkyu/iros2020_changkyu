#include "camera/camera_utils.hpp"

#include <opencv2/opencv.hpp>

using namespace std;
using namespace pcl;
using namespace changkyu;

CameraManager* cam;

const int ESC=27;
const int SPACE=32;
const string camera_name("camera2");

int main(int argc, char* argv[])
{
    // ROS init
    ros::init(argc,argv,"changkyu_camera_recorder_app");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    cam = new CameraManager(nh, 0.005);
    
    cv::namedWindow("color");
    cv::moveWindow("color", 0,0);
    cv::namedWindow("depth");
    cv::moveWindow("depth", 640,0);

    int idx = 0;
    bool recording = false;
    bool running = true;
    while(running)
    { 
        cv::Mat image, depth;
        cam->GetInputFromCamera(camera_name, image, depth);
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

        cv::imshow("color",image);
        cv::imshow("depth",depth);
        int key = cv::waitKey(100) & 0xFF;
        if( key == ESC )
        {
            running=false;
            break;
        } 
        else if( key == SPACE )
        {
            if( recording==false )
            {
                recording = true;
                char prefix[256];
                sprintf(prefix,"/home/cs1080/test%d",idx++);
                cam->StartRecording(camera_name, prefix, 0.1);
            }
            else
            {
                recording = false;
                cam->StopRecording();
            }
        }
        else if( key == 255 )
        {
            // Ignore
        }
        else
        {
            cout << "key: \"" << key << "\"" << endl;
        }
    }

    delete cam;
    ros::shutdown();    
}