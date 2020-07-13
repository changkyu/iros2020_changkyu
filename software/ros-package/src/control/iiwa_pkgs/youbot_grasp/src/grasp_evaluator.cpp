#include "youbot_grasp/grasp_evaluator.h"

/*constructor*/
GraspEvaluator::GraspEvaluator()
{

}

/*destructor*/
GraspEvaluator::~GraspEvaluator()
{

}

bool compare_quality(const plannedGrasp* first, const plannedGrasp* second)
{
  if(first->get_quality() <= second->get_quality())
  {
    return true;
  }else{
    return false;
  } 
}

/*evaluate grasps by assigning each candidate grasps a grasp quality. The quality represents the easiness of grasping the target object using the candidate grasp
    object_file_name, the filename of the target STL file
    grasp_list, the candidate grasp point list, each entry has a type of plannedGrasp, defined in include/grasp_grasps.h
    radius, radius of the suction cup
*/    
bool GraspEvaluator::evaluate(std::string object_file_name, std::list<plannedGrasp*> & grasp_list, std::string gripper_type, double radius)
{
	MyMesh m, res;          
    int LoadMask;                  
    float length;                               // length of the intersection between the ball and the mesh
    float tolerance = 0.5;  
    float ratio;                               // ratio between the length of the intersection and the length of the suction cup
    int grasp_num = 0;
    vcg::Point3f com;
    double grasp_quality = 0;   
    tf::Vector3 grasp_point, grasp_dir;
    if(vcg::tri::io::ImporterOBJ<MyMesh>::Open(m,(ros::package::getPath("youbot_grasp")+"/models/"+object_file_name+".obj").c_str(),LoadMask)!=0)
    {  
        
        // if(access(object_file_name.c_str(), F_OK)!=-1){
        //         printf("file exists\n");
        // }else{
        //         printf("file %s doesn't exists\n", object_file_name.c_str());

        // }
       
       printf("Error reading file  %s\n",object_file_name.c_str());
       exit(0);
    } 
  	vcg::tri::UpdateNormal<MyMesh>::PerFace(m);
    com = get_COM(m);
    com[0] = com[0]/1000;
    com[1] = com[1]/1000;
    com[2] = com[2]/1000;
    std::cout<<"center of mass is"<< com[0] << ',' << com[1] << ',' << com[2] << std::endl;
    std::list<plannedGrasp*>::iterator i;
    for(i = grasp_list.begin(); i != grasp_list.end(); i ++)
    {

    	grasp_point = (*i)->get_graspDirection().get_point();
    	grasp_dir = (*i)->get_graspDirection().get_dir();
        if(gripper_type == "suction_cup")
        {  
    	  grasp_quality = sqrt(pow(grasp_point[0] - com[0], 2) + pow(grasp_point[1] - com[1], 2)) / radius;
    	}
        else if(gripper_type == "standard" )
        {
          grasp_quality = sqrt(pow(grasp_point[0] - com[0], 2) + pow(grasp_point[1] - com[1], 2) + pow(grasp_point[2] - com[2], 2));
        }
        (*i)->set_quality(grasp_quality);
        std::cout<<grasp_quality << ' ';
    }
    grasp_list.sort(compare_quality);
}


/* Calculate the center of mass of the mesh model */
vcg::Point3f GraspEvaluator::get_COM(MyMesh & m){

	vcg::Point3f com(0,0,0);
	vcg::Point3f cof;
	double totalArea = 0;
	for(int i = 0; i < m.FN(); i ++)
	{
		cof = vcg::Barycenter(m.face[i]);
		double planeArea = 2 * vcg::DoubleArea(m.face[i]);  
		com[0] += cof[0]*planeArea;
		com[1] += cof[1]*planeArea;
		com[2] += cof[2]*planeArea;
		totalArea += planeArea;
	}
	com[0] = com[0] / totalArea;
	com[1] = com[1] / totalArea;
	com[2] = com[2] / totalArea;
	return com;
}
