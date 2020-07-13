#include "youbot_grasp/approach_dir_generator.h"


sampledPoint::sampledPoint(geometry_msgs::Point sample_point, geometry_msgs::Point sample_normal){
  sample_point_ = sample_point;
  sample_normal_ = sample_normal;
}


/*constructor*/
ApproachDirGenerator::ApproachDirGenerator()
{

}

/*destructor*/
ApproachDirGenerator::~ApproachDirGenerator()
{

}

bool compare_quality(const sampledPoint* first, const sampledPoint* second)
{
  if(first->quality <= second->quality)
  {
    return true;
  }else{
    return false;
  } 
}

/*generate approaching direction
    object_file_name, filename of the target STL file
    sample_point, pointer to the vector of points
    sample_normal, pointer to the vector of the unit normals of each point
*/
void ApproachDirGenerator::generateApproachDir(std::string object_file_name, geometry_msgs::Pose target_pose, std::list<sampledPoint*> &sampledPoint_list, std::vector<geometry_msgs::Point>* sample_point, std::vector<geometry_msgs::Point>* sample_normal)
{
  MyMesh m;
  int load_mask;
  double total_surface = 0;
  double average_surface;
  int total_sample_num = 600;
  float sample_scale;
  vcg::Point3f com;
  if(object_file_name == "cube"){
    std::vector<int> index_list;
    index_list.push_back(5);
    index_list.push_back(4);
    index_list.push_back(6);
    index_list.push_back(3);
    index_list.push_back(7);
    index_list.push_back(2);
    index_list.push_back(8);
    index_list.push_back(1);
    index_list.push_back(9);
    // index_list.push_back(1);
    // index_list.push_back(2);
    // index_list.push_back(3);
    // index_list.push_back(4);
    // index_list.push_back(5);
    // index_list.push_back(6);
    // index_list.push_back(7);
    // index_list.push_back(8);
    // index_list.push_back(9);
    double x_min = -0.125;
    double x_max = 0.125;
    double y_min = -0.025;
    double y_max = 0.025;
    double z_min = -0.025;
    double z_max = 0.025;

// these sample point and dir are normal to the face
    for(int i = 1; i < 10; i++){
        double x_step = (x_max - x_min)/10.0;
        geometry_msgs::Point point;
        point.x = x_max - index_list[i-1]*x_step;
        point.y = 0;
        point.z = z_max;
        ROS_INFO_STREAM("sample_point:"<<point.x<<","<<point.y<<","<<point.z);
        geometry_msgs::Point normal;
        normal.x = 0;
        normal.y = 0;
        normal.z = 1;
        ROS_INFO_STREAM("sample_normal:"<<normal.x<<","<<normal.y<<","<<normal.z);
        sampledPoint *sp = new sampledPoint(point, normal); 
        sampledPoint_list.push_back(sp);
        normal.x = point.x * 5;
        normal.y = 0;
        normal.z = sqrt(1 - pow(normal.x, 2));
        ROS_INFO_STREAM("sample_normal:"<<normal.x<<","<<normal.y<<","<<normal.z);
        sp = new sampledPoint(point, normal); 
        sampledPoint_list.push_back(sp);
    }
    for(int i = 1; i < 10; i++){
        double x_step = (x_max - x_min)/10;
        geometry_msgs::Point point;
        point.x = x_min + index_list[i-1]*x_step;
        point.y = 0;
        point.z = z_min;
        ROS_INFO_STREAM("sample_point:"<<point.x<<","<<point.y<<","<<point.z);
        geometry_msgs::Point normal;
        normal.x = 0;
        normal.y = 0;
        normal.z = -1;
        ROS_INFO_STREAM("sample_normal:"<<normal.x<<","<<normal.y<<","<<normal.z);
        sampledPoint *sp = new sampledPoint(point, normal); 
        sampledPoint_list.push_back(sp);
        normal.x = point.x * 5;
        normal.y = 0;
        normal.z = -sqrt(1 - pow(normal.x, 2));
        sp = new sampledPoint(point, normal); 
        sampledPoint_list.push_back(sp);
    }
    for(int i = 1; i < 10; i++){
        double x_step = (x_max - x_min)/10;
        geometry_msgs::Point point;
        point.x = x_min + index_list[i-1]*x_step;
        point.y = y_max;
        point.z = 0;
        ROS_INFO_STREAM("sample_point:"<<point.x<<","<<point.y<<","<<point.z);
        geometry_msgs::Point normal;
        normal.x = 0;
        normal.y = 1;
        normal.z = 0;
        ROS_INFO_STREAM("sample_normal:"<<normal.x<<","<<normal.y<<","<<normal.z);
        sampledPoint *sp = new sampledPoint(point, normal); 
        sampledPoint_list.push_back(sp);
        normal.x = point.x * 5;
        normal.y = sqrt(1 - pow(normal.x, 2));
        normal.z = 0;
        sp = new sampledPoint(point, normal); 
        sampledPoint_list.push_back(sp);
    }
    for(int i = 1; i < 10; i++){
        double x_step = (x_max - x_min)/10;
        geometry_msgs::Point point;
        point.x = x_min + index_list[i-1]*x_step;
        point.y = y_min;
        point.z = 0;
        ROS_INFO_STREAM("sample_point:"<<point.x<<","<<point.y<<","<<point.z);
        geometry_msgs::Point normal;
        normal.x = 0;
        normal.y = -1;
        normal.z = 0;
        ROS_INFO_STREAM("sample_normal:"<<normal.x<<","<<normal.y<<","<<normal.z);
        sampledPoint *sp = new sampledPoint(point, normal); 
        sampledPoint_list.push_back(sp);
        normal.x = point.x * 5;
        normal.y = -sqrt(1 - pow(normal.x, 2));
        normal.z = 0;
        sp = new sampledPoint(point, normal); 
        sampledPoint_list.push_back(sp);
    }
    geometry_msgs::Point point;
    point.x = x_min;
    point.y = 0;
    point.z = 0;
    ROS_INFO_STREAM("sample_point:"<<point.x<<","<<point.y<<","<<point.z);
    geometry_msgs::Point normal;
    normal.x = -1;
    normal.y = 0;
    normal.z = 0;
    ROS_INFO_STREAM("sample_normal:"<<normal.x<<","<<normal.y<<","<<normal.z);
    sampledPoint *sp = new sampledPoint(point, normal); 
    sampledPoint_list.push_back(sp);
    point.x = x_max;
    point.y = 0;
    point.z = 0;
    ROS_INFO_STREAM("sample_point:"<<point.x<<","<<point.y<<","<<point.z);
    
    normal.x = 1;
    normal.y = 0;
    normal.z = 0;
    ROS_INFO_STREAM("sample_normal:"<<normal.x<<","<<normal.y<<","<<normal.z);
    sp = new sampledPoint(point, normal); 
    sampledPoint_list.push_back(sp);
    
// below sample point and their dir are not normal anymore


  }else{ 
  

  if(vcg::tri::io::ImporterOBJ<MyMesh>::Open(m,(ros::package::getPath("youbot_grasp")+"/models/"+ object_file_name + ".obj").c_str(),load_mask)!=0)
  {  
    printf("Error reading file  %s\n",object_file_name.c_str());
    exit(0);
  } 

    /*  One way of sampling surface points  */
    // vcg::tri::SurfaceSampling<MyMesh,vcg::tri::TrivialSampler<MyMesh> >::SamplingRandomGenerator().initialize(time(0));

    // vector<vcg::Point3f> pointVec;
    // int sampleNum = 300;
    // float rad = 0.0;
    // vcg::tri::PoissonSampling<MyMesh>(m,pointVec,sampleNum,rad);
    // std::cout << "total: " << pointVec.size() << std::endl;
    // for (int i = 0; i < pointVec.size(); i ++)
    // {
    // 	cout<<"sample point: " << i << " "<< pointVec[i][0] << " " << pointVec[i][1] << " " << pointVec[i][2] << std::endl;
    //     geometry_msgs::Point point;
    //     point.x = pointVec[i][0];
    //     point.y = pointVec[i][1];
    //     point.z = pointVec[i][2];
    //     sample_point->push_back(point);
    // }
  
  // sample points from each face of the mesh by numbers proportional to their area
  vcg::tri::UpdateNormal<MyMesh>::PerVertexNormalizedPerFaceNormalized(m);
  vcg::tri::UpdateBounding<MyMesh>::Box(m);
  std::cout<<m.VN()<<"vertexs"<<m.FN()<<"faces"<<m.EN()<<"edges"<<endl;
  for(int i = 0;i < m.FN();i++)
  {
    total_surface += triangle_area(m.face[i].V(0)->P(), m.face[i].V(1)->P(), m.face[i].V(2)->P());
  }
  // average_surface = total_surface / m.FN();
  // sample_scale = 1;
  // if( average_surface < 1)
  // {
  //   sample_scale = 1/average_surface; 
  // }
  // else if( average_surface > 3)
  // {
  //   sample_scale = 1/average_surface;
  // }
    
  for(int i = 0;i < m.FN();i++) //browse through each face of the mesh
  {
    std::vector<vcg::Point3f> sampleVec;
    vcg::Plane3f pl;
    pl.Init(vcg::Barycenter(m.face[i]),m.face[i].N());
    double surface = triangle_area(m.face[i].V(0)->P(), m.face[i].V(1)->P(), m.face[i].V(2)->P());
    int sample_nums = total_sample_num * surface / total_surface;
    ROS_INFO_STREAM("sample num:"<<sample_nums);
    // if(surface < 1){
    //   sample_nums = ceil(surface * sample_scale * 3); 
    // }else{
    //   sample_nums = ceil(surface * sample_scale);
    // }
    com = get_COM(m);
    com[0] = com[0]/1000;
    com[1] = com[1]/1000;
    com[2] = com[2]/1000;
    for(int j = 0; j < sample_nums; j++) //  the sample numbers is proportional to the area of the sampled triangle.
    {
      vcg::Point3f p = vcg::tri::SurfaceSampling<MyMesh>::RandomPointInTriangle(m.face[i]);
      // std::cout<<p[0]<<"--"<<p[1]<<"--"<<p[2]<<endl; 
      vcg::Point3f n = m.face[i].N();
      geometry_msgs::Point point;
      point.x = p[0]/1000;
      point.y = p[1]/1000;
      point.z = p[2]/1000;
      ROS_INFO_STREAM("sample_point:"<<point.x<<","<<point.y<<","<<point.z);
      sample_point->push_back(point);
      geometry_msgs::Point normal;
      normal.x = n[0];
      normal.y = n[1];
      normal.z = n[2];
      ROS_INFO_STREAM("sample_normal:"<<normal.x<<","<<normal.y<<","<<normal.z);



      sample_normal->push_back(normal);
      sampledPoint *sp = new sampledPoint(point, normal); 
      sampledPoint_list.push_back(sp);
      // double length = sqrt(pow(point.x - com[0], 2) + pow(point.y - com[1], 2) + pow(point.z - com[2], 2));
      // normal.x = (point.x - com[0]) / length;
      // normal.y = (point.y - com[1]) / length;
      // normal.z = (point.z - com[2]) / length;
      // sp = new sampledPoint(point, normal);
      // ROS_INFO_STREAM("sample_point:"<<point.x<<","<<point.y<<","<<point.z);
      // ROS_INFO_STREAM("sample_normal:"<<normal.x<<","<<normal.y<<","<<normal.z);
      // sampledPoint_list.push_back(sp);
    }

  }

  // sort the sample point and normal based on their distance to the mass center
  com = get_COM(m);
  com[0] = com[0]/1000;
  com[1] = com[1]/1000;
  com[2] = com[2]/1000;
  tf::Quaternion target_q(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
  tf::Matrix3x3 target_m(target_q);
  tf::Vector3 x_axis = target_m.getColumn(0);
  // if(x_axis.getZ() < -0.1){
  //   com[0] += 0.05;
  // }else if(x_axis.getZ() > 0.1){
  //   com[0] -= 0.05;
  // }
  double grasp_quality;
  geometry_msgs::Point test_point, test_normal;
  std::cout<<"center of mass is"<< com[0] << ',' << com[1] << ',' << com[2] << std::endl;
  std::list<sampledPoint*>::iterator i, j;
  for(i = sampledPoint_list.begin(); i != sampledPoint_list.end(); i ++)
  {

      test_point = (*i)->sample_point_;
      test_normal = (*i)->sample_normal_;
      // double AB_length = sqrt(pow(test_point.x - com[0], 2) + pow(test_point.y - com[1], 2) + pow(test_point.z - com[2], 2));
      // double normal_length = sqrt(pow(test_normal.x, 2) + pow(test_normal.y, 2) + pow(test_normal.z, 2));
      // double inner_product = (test_point.x - com[0])*test_normal.x + (test_point.y - com[1])*test_normal.y + (test_point.z - com[2])*test_normal.z;
      // double cos_theta = inner_product / (AB_length*normal_length);
      // double sin_theta = sqrt(1 - pow(cos_theta,2));
      // grasp_quality = AB_length * sin_theta;
      double x_offset = pow(test_point.x - com[0], 2);
      double y_offset = pow(test_point.y - com[1], 2);
      double z_offset = pow(test_point.z - com[2], 2);
      if(x_offset > y_offset && x_offset > z_offset){
        grasp_quality = sqrt(pow(test_point.y - com[1], 2) + pow(test_point.z - com[2], 2));
      }else if(y_offset > x_offset && y_offset > z_offset){
        grasp_quality = sqrt(pow(test_point.x - com[0], 2) + pow(test_point.z - com[2], 2));
      }else if(z_offset > x_offset && z_offset > y_offset){
        grasp_quality = sqrt(pow(test_point.x - com[0], 2) + pow(test_point.y - com[1], 2));
      }
    //  grasp_quality = sqrt(pow(test_point.x - com[0], 2) + pow(test_point.y - com[1], 2) + pow(test_point.z - com[2], 2));
        
      (*i)->quality = grasp_quality;
  }
  sampledPoint_list.sort(compare_quality);
  geometry_msgs::Point special_point, special_normal;
  for(j = sampledPoint_list.begin(); j != sampledPoint_list.end(); j ++){
      test_point = (*j)->sample_point_;
      test_normal = (*j)->sample_normal_;
      if(test_normal.z == 1){
        if(abs(test_point.x) > abs(test_point.y)){
          special_point.x = test_point.x;
          special_point.y = 0;
          special_point.z = test_point.z;
          special_normal.y = 0;
          if(test_point.x > 0){
            for(double nx = 0.1; nx < 0.9; nx += 0.1){
              double nz = sqrt(1- pow(nx,2));
              special_normal.x = nx;
              special_normal.z = nz;
              sampledPoint *sp = new sampledPoint(special_point, special_normal); 
              sampledPoint_list.push_front(sp);

            }
          }else{
            for(double nx = -0.1; nx > -0.9; nx -= 0.1){
              double nz = sqrt(1- pow(nx,2));
              special_normal.x = nx;
              special_normal.z = nz;
              sampledPoint *sp = new sampledPoint(special_point, special_normal); 
              sampledPoint_list.push_front(sp);

            }
          }
        }else if(abs(test_point.x) < abs(test_point.y)){
          special_point.x = 0;
          special_point.y = test_point.y;
          special_point.z = test_point.z;
          special_normal.x = 0;
          if(test_point.y > 0){
            for(double ny = 0.1; ny < 0.9; ny += 0.1){
              double nz = sqrt(1- pow(ny,2));
              special_normal.y = ny;
              special_normal.z = nz;
              sampledPoint *sp = new sampledPoint(special_point, special_normal); 
              sampledPoint_list.push_front(sp);

            }
          }else{
            for(double ny = -0.1; ny > -0.9; ny -= 0.1){
              double nz = sqrt(1- pow(ny,2));
              special_normal.y = ny;
              special_normal.z = nz;
              sampledPoint *sp = new sampledPoint(special_point, special_normal); 
              sampledPoint_list.push_front(sp);

            }
          }
        }
      }
  }
  


  ROS_INFO_STREAM("sample num:"<< sample_normal->size());
}
}
/*
Calculate the area of the triangle with vertexes a, b, c
*/
double ApproachDirGenerator::triangle_area(vcg::Point3f a, vcg::Point3f b, vcg::Point3f c)
{
  double x1, x2, x3, y1, y2, y3;
  double surface;
  x1 = a[0] - b[0];
  x2 = a[1] - b[1];
  x3 = a[2] - b[2];
  y1 = a[0] - c[0];
  y2 = a[1] - c[1];
  y3 = a[2] - c[2];
  surface = sqrt(pow(x2*y3 - x3*y2, 2) + pow(x3*y1 - x1*y3, 2) + pow(x1*y2 - x2*y1, 2))/2 ;
  return surface;
}

vcg::Point3f ApproachDirGenerator::get_COM(MyMesh & m){

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