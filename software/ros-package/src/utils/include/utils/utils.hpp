#ifndef _UTILS__HPP_
#define _UTILS__HPP_

#include <set>
#include <map>
#include <vector>

#include <opencv2/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

#include <time.h>

namespace utils
{

extern uint8_t colors_vis[][3];

float IntersectionOverUnion(const pcl::PolygonMesh &polymesh1,
                            const pcl::PolygonMesh &polymesh2, 
                            const float resolution=0.005);

template<typename PointT>
float compute_dist(PointT &pt1, PointT &pt2);

template<typename Scalar> Scalar UnitRandom(const Scalar min=0, const Scalar max=1);
template<typename Scalar> Eigen::Quaternion<Scalar> UnitRandom_Quaternion();

void UnitRandom_NoDup( const int min, const int max, const size_t n_output, 
                       std::vector<int> &numbers_out );

void UnitRandom_CameraExtrinsic( 
    Eigen::Matrix4f &cam_extrinsic,
    const std::pair<float,float> d_range,
    const std::pair<float,float> a_range = std::pair<float,float>(0,M_PI*2   ),
    const std::pair<float,float> e_range = std::pair<float,float>(M_PI*0.125, 
                                                                  M_PI*0.375));

template<typename PointT>
void PolygonMesh2PointCloud( const pcl::PolygonMesh &polymesh, 
                             pcl::PointCloud<PointT> &cloud,
                             const float resolution=0.005     );

template<typename PointT>
void FillPolygonMesh( const pcl::PolygonMesh &polymesh, 
                      pcl::PointCloud<PointT> &cloud_fill, 
                      const float resolution=0.005        );

template<typename PointT>
void PointClouds2LabelImage(
    const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds,
    cv::Mat &image, const size_t width, const size_t height,
    const Eigen::Matrix3f cam_K, const Eigen::Matrix4f cam_RT,
    const std::vector<int> &labels_in=std::vector<int>() );

template<typename PointT>
void PointCloud2Depth( const pcl::PointCloud<PointT> &cloud,
                       cv::Mat &depth, const size_t width, const size_t height,
                       const Eigen::Matrix3f cam_K, const Eigen::Matrix4f cam_RT);

template<typename PointT, typename TYPE_DEPTH>
void Depth2PointCloud( const cv::Mat &depth,
                       const Eigen::Matrix3f &cam_K, 
                       const Eigen::Matrix4f &cam_RT,
                       pcl::PointCloud<PointT> &cloud );

template<typename PointT, typename TYPE_DEPTH>
void PointCloudfromDepth( pcl::PointCloud<PointT> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,
                          const cv::Mat &mask,
                          const bool isOrganized=false    );

template<typename PointT, typename TYPE_DEPTH>
void PointCloudfromDepth( pcl::PointCloud<PointT> &cloud,
                          const cv::Mat &depth,
                          const float depth_scale,
                          const std::vector<float> &cam_in,
                          const std::vector<float> &cam_ex,                          
                          const cv::Mat &image,
                          const cv::Mat &mask,
                          const bool isOrganized=false    );

template<typename PointT, typename TYPE_DEPTH>
void PointCloudsfromDepth( std::map<uint8_t,typename pcl::PointCloud<PointT>::Ptr> &clouds,
                           const cv::Mat &seg,
                           const cv::Mat &depth,
                           const float depth_scale,
                           const std::vector<float> &cam_in,
                           const std::vector<float> &cam_ex,                          
                           const cv::Mat &image,
                           const cv::Mat &mask              );

typedef std::vector<std::vector<std::vector<int> > >  voxel_t;
typedef std::vector<std::vector<std::vector<float> > > tsdf_t;

template<typename PointT>
void PointClouds2Voxel( const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds,
                        voxel_t &voxels, PointT &voxel_origin,
                        int &n_x, int &n_y, int &n_z,
                        const float resolution=0.005,
                        const std::vector<int> labels_in=std::vector<int>()   );

template<typename PointT>
void PointCloud2Voxel( const pcl::PointCloud<PointT> &cloud,
                       voxel_t &voxels, PointT &voxel_origin,
                       int &n_x, int &n_y, int &n_z,
                       const float resolution=0.005,
                       const int label=1                                      );

template<typename PointT>
void PointCloud2PointCloudVox( const pcl::PointCloud<PointT> &cloud_in,
                               pcl::PointCloud<PointT> &cloud_vox,
                               const float resolution                  );

template<typename PointT>
void Voxel2PointClouds( const voxel_t &voxel, 
                        std::map<uint8_t,typename pcl::PointCloud<PointT>::Ptr> &clouds,
                        const float resolution=0.005 );

template <typename Scalar>
void Voxel2PointCloud( 
    const std::vector<std::vector<std::vector<Scalar> > > &voxel,
    pcl::PointCloud<pcl::PointXYZRGB> &cloud, const float size=0.005,
    const float val_min=INFINITY, const float val_max=-INFINITY   );

void Voxel2PolygonMeshs( const voxel_t &voxel,
                         const float resolution,
                         std::map<uint8_t,pcl::PolygonMesh> &polymeshs );

template<typename PointT, typename TYPE_DEPTH>
void VoxelsfromDepth( voxel_t &voxel, 
                      PointT &voxel_origin,
                      int &n_x, int &n_y, int &n_z,
                      const float resolution,                      
                      std::map<uint8_t,typename pcl::PointCloud<PointT>::Ptr> &seg2cloud,
                      std::map<uint8_t,typename pcl::PointCloud<PointT>::Ptr> &seg2hidden,                      
                      const cv::Mat &seg,
                      const cv::Mat &depth,
                      const float depth_scale,
                      const std::vector<float> &cam_in,
                      const std::vector<float> &cam_ex,
                      const cv::Mat &mask                );

void GetTSDF( const std::vector<pcl::PolygonMesh> &polymeshs_in, 
              const std::vector<Eigen::Matrix4f> &poses,
              voxel_t &voxel, tsdf_t &tsdf,
              int &n_x, int &n_y, int &n_z, 
              float &offset_x, float &offset_y, float &offset_z,
              const float resolution,
              const std::vector<int> labels=std::vector<int>()           );

template<typename PointT>
void GetTSDF( const typename pcl::PointCloud<PointT>::Ptr &cloud_surf_in, 
              voxel_t &voxel, tsdf_t &tsdf,
              int &n_x, int &n_y, int &n_z,
              float &offset_x, float &offset_y, float &offset_z, 
              const float resolution,
              const std::vector<int> labels_in              );

template<typename PointT>
void GetTSDF( const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds_surf_in, 
              voxel_t &voxel, tsdf_t &tsdf,
              int &n_x, int &n_y, int &n_z, 
              float &offset_x, float &offset_y, float &offset_z,
              const float resolution,
              const std::vector<int> labels=std::vector<int>()           );

void WriteVoxLabel_sscnet(const std::string &filename, 
                          std::vector<float> &vox_origin, 
                          std::vector<float> &cam_pose, voxel_t &voxel );

void ReadVoxLabel_sscnet( const std::string &filename, 
                          std::vector<float> &vox_origin, 
                          std::vector<float> &cam_pose, 
                          voxel_t &voxel,
                          const int n_x, const int n_y, const int n_z );

void FillHole( const cv::Mat &src, cv::Mat &dst, const int size=10 );

template<typename Scalar>
std::set<Scalar> Unique(const cv::Mat& input);

float float16tofloat32(char* data);

template<typename PointT>
void FindCorners2D( const pcl::PointCloud<PointT> &cloud_prj,
                    std::vector<PointT> &pts_corner);

template<typename PointT>
void FindBoundary2D( const typename pcl::PointCloud<PointT>::Ptr cloud_prj,
                     std::vector<uint32_t> &idxes_boundary,
                     pcl::PointCloud<PointT> &cloud_boundary,
                     const float resolution=0.005);

template<typename PointT>
void FindBoundary2D( const typename pcl::PointCloud<PointT>::Ptr cloud_prj,
                     pcl::PointCloud<PointT> &cloud_boundary,
                     const float resolution=0.005);

template<typename PointT>
void FindBoundary2D( const typename pcl::PointCloud<PointT>::Ptr cloud_prj,
                     std::vector<uint32_t> &idxes_boundary,
                     const float resolution=0.005);

void AverageQuaternion( std::vector<Eigen::Quaternionf> &qs, 
                        Eigen::Quaternionf &mean );

}

#endif