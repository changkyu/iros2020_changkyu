#include <iostream>
#include <boost/program_options.hpp>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/convex_hull.h>

#include "utils/utils.hpp"
#include "utils/utils_python.hpp"
#include "utils/utils_visualization.hpp"

using namespace std;
using namespace pcl;
using namespace utils;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

void Evaluation( voxel_t &voxel_pred, voxel_t &voxel_gt, voxel_t &voxel_region  )
{
    int n_x = voxel_gt.size();
    int n_y = voxel_gt[0].size();
    int n_z = voxel_gt[0][0].size();

    set<int> labels;
    
    int tp = 0, fp = 0, fn = 0;
    for( int i_x=0; i_x<n_x; i_x++ )
    for( int i_y=0; i_y<n_y; i_y++ )
    for( int i_z=0; i_z<n_z; i_z++ )
    {
        if( voxel_region[i_x][i_y][i_z] == 0 ) continue;

        bool occ = false;        
        if( voxel_pred[i_x][i_y][i_z] > 0 ) occ = true;
        
        if( occ )
        {
            if( voxel_gt[i_x][i_y][i_z] )
            {
                tp++;
            }
            else
            {
                fp++;
            }
        }
        else
        {
            if( voxel_gt[i_x][i_y][i_z] )
            {
                fn++;
            }
        }

        if( voxel_gt[i_x][i_y][i_z] > 0 )
            labels.insert( voxel_gt[i_x][i_y][i_z] );
    }

    float precision = ((float)tp) / ((float)(tp + fp));
    float recall    = ((float)tp) / ((float)(tp + fn));
    float IoU       = ((float)tp) / ((float)(fn + tp + fp));

    //printf("IoU\tPrecision\tRecall\n");
    printf("all\t%.2f\t%.2f\t%.2f\n", IoU*100, precision*100, recall*100);
    set<int> set_label;
    for( set<int>::iterator it=labels.begin(); it!=labels.end(); it++ )
    {
        int label = *it;

        int tp = 0, fp = 0, fn = 0;
        for( int i_x=0; i_x<n_x; i_x++ )
        for( int i_y=0; i_y<n_y; i_y++ )
        for( int i_z=0; i_z<n_z; i_z++ )
        {
            if( voxel_region[i_x][i_y][i_z] == 0 ) continue;

            if( voxel_pred[i_x][i_y][i_z] == label )
            {
                if( voxel_gt[i_x][i_y][i_z] == label )
                {
                    tp++;
                }
                else
                {
                    fp++;
                }
            }
            else
            {
                if( voxel_gt[i_x][i_y][i_z] == label )
                {
                    fn++;
                }
            }
        }

        float precision = ((float)tp) / ((float)(tp + fp));
        float recall    = ((float)tp) / ((float)(tp + fn));
        float IoU       = ((float)tp) / ((float)(fn + tp + fp));
        
        printf("%d\t%.2f\t%.2f\t%.2f\n", label, IoU*100, precision*100, recall*100);

    }
}

int main(int argc, char* argv[])
{   
    string fp_input;
    string fp_eval;
    string fp_gt;
    po::options_description desc("Example Usage");
    desc.add_options()
        ("help", "help")
        ("input,i",   po::value<string>(&fp_input),"input file")        
        ("gt,g",   po::value<string>(&fp_gt),"ground truth file")        
        ("eval,e",   po::value<string>(&fp_eval),"eval region file")        
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if( vm.count("help")        ||        
        fp_input.compare("")==0    ) 
    {        
        cout << desc << "\n";
        return 0;
    }

    fs::path path_input(fp_input);
    if( !fs::is_regular_file(path_input) )
    {
        cerr << "[Error] Invalid file: " << path_input.string() << endl;
        return 0;
    }

    string name = fp_input.substr(0,fp_input.length()-4);

    const float resolution = 0.005;
    voxel_t voxel;
    LoadVoxel2Numpy(fp_input.c_str(),voxel );

    /*
    utils::tsdf_t tsdf_gt;
    LoadTSDF2Numpy(fp_gt.c_str(), tsdf_gt);
    
    voxel_t voxel_gt;
    voxel_gt.resize(tsdf_gt.size());
    for( int i_x=0; i_x<tsdf_gt.size(); i_x++ )
    {
        voxel_gt[i_x].resize(tsdf_gt[i_x].size());
        for( int i_y=0; i_y<tsdf_gt[i_x].size(); i_y++ ) 
        {
            voxel_gt[i_x][i_y].resize(tsdf_gt[i_x][i_y].size());
            for( int i_z=0; i_z<tsdf_gt[i_x][i_y].size(); i_z++ )
            {
                voxel_gt[i_x][i_y][i_z] = tsdf_gt[i_x][i_y][i_z] < 0;
            }
        }
    }
    */
    
    utils::voxel_t voxel_gt;
    if( fp_gt.substr(fp_gt.size()-4).compare(".npy")==0 )
    {
        LoadVoxel2Numpy(fp_gt.c_str(), voxel_gt);
    }
    else if( fp_gt.substr(fp_gt.size()-4).compare(".dat")==0 )
    {
        ifstream ifs(fp_gt);
        voxel_gt.resize(voxel.size());
        for( int i_x=0; i_x<voxel.size(); i_x++ )
        {
            voxel_gt[i_x].resize(voxel[i_x].size());
            for( int i_y=0; i_y<voxel[i_x].size(); i_y++ )
            {
                voxel_gt[i_x][i_y].resize(voxel[i_x][i_y].size());
                for( int i_z=0; i_z<voxel[i_x][i_y].size(); i_z++ )
                {
                    uint16_t num16;
                    ifs.read((char*)&num16,2);
                    float val = utils::float16tofloat32((char*)&num16);

                    int occ = 0;
                    if( val != val )   occ = 1;
                    else if( val < 0 ) occ = 1;
                    else               occ = 0;

                    voxel_gt[i_x][i_y][i_z] = occ;
                }
            }
        }
    }
    else
    {
        cerr << "[Error] Invalid file: " << fp_gt << endl;
    }

    voxel_t eval;
    LoadBoolVoxelfromMat(fp_eval.c_str(),eval,"evaluation_region");

    Evaluation(voxel, voxel_gt, eval );

    map<uint8_t,PointCloud<PointXYZRGB>::Ptr> clouds;    
    Voxel2PointClouds<PointXYZRGB>( voxel, clouds, resolution );

    for( map<uint8_t,PointCloud<PointXYZRGB>::Ptr>::iterator 
         it = clouds.begin(); it != clouds.end(); it++ )
    {
        for( int p=0; p<it->second->size(); p++ )
        {
            uint8_t r = utils::colors_vis[it->first%20][0];
            uint8_t g = utils::colors_vis[it->first%20][1];
            uint8_t b = utils::colors_vis[it->first%20][2];

            it->second->points[p].r = r;
            it->second->points[p].g = g;
            it->second->points[p].b = b;            
        }

        PolygonMesh polymesh;
        ConvexHull<PointXYZRGB> chull;
        chull.setDimension(3);
        chull.setInputCloud (it->second);
        chull.reconstruct (polymesh);
        
        stringstream ss;
        ss << (int)it->first;
        string fp_save = name + "." + ss.str() + ".ply";
        io::savePLYFile (fp_save.c_str(), polymesh);
        //cout << fp_save << endl;
    }

    map<uint8_t,PointCloud<PointXYZRGB>::Ptr> clouds_gt;    
    Voxel2PointClouds<PointXYZRGB>( voxel_gt, clouds_gt, resolution );

    for( map<uint8_t,PointCloud<PointXYZRGB>::Ptr>::iterator 
         it = clouds_gt.begin(); it != clouds_gt.end(); it++ )
    {
        for( int p=0; p<it->second->size(); p++ )
        {
            uint8_t r = utils::colors_vis[it->first%20][0];
            uint8_t g = utils::colors_vis[it->first%20][1];
            uint8_t b = utils::colors_vis[it->first%20][2];

            it->second->points[p].r = r;
            it->second->points[p].g = g;
            it->second->points[p].b = b;            
        }

        PolygonMesh polymesh;
        ConvexHull<PointXYZRGB> chull;
        chull.setDimension(3);
        chull.setInputCloud (it->second);
        chull.reconstruct (polymesh);
        
        stringstream ss;
        ss << (int)it->first;
        string fp_save = name + "." + ss.str() + ".gt.ply";
        io::savePLYFile (fp_save.c_str(), polymesh);
        //cout << fp_save << endl;
    }

    utils::voxel_t voxel_diff;
    {        
        voxel_diff.resize(voxel.size());
        for( int i_x=0; i_x<voxel.size(); i_x++ )
        {
            voxel_diff[i_x].resize(voxel[i_x].size());
            for( int i_y=0; i_y<voxel[i_x].size(); i_y++ )
            {
                voxel_diff[i_x][i_y].resize(voxel[i_x][i_y].size());
                for( int i_z=0; i_z<voxel[i_x][i_y].size(); i_z++ )
                {
                    if( eval[i_x][i_y][i_z] > 0 )
                    {
                        if( (voxel[i_x][i_y][i_z] >  0 && voxel_gt[i_x][i_y][i_z] <= 0) ||
                            (voxel[i_x][i_y][i_z] <= 0 && voxel_gt[i_x][i_y][i_z] >  0)    )
                        {
                            voxel_diff[i_x][i_y][i_z] = 1;
                        }
                        else
                        {
                            voxel_diff[i_x][i_y][i_z] = 0;
                        }
                    }                         
                }
            }
        }
    }

#if 0
    visualization::PCLVisualizer viewer;
    int v1,v2,v3, v4;
    viewer.createViewPort(0.00,0.00,0.50,0.50,v1);
    viewer.createViewPort(0.50,0.00,1.00,0.50,v2);
    viewer.createViewPort(0.00,0.50,0.50,1.00,v3);
    viewer.createViewPort(0.50,0.50,1.00,1.00,v4);
    viewer.setWindowName("debug");
    viewer.setSize(1000,1000);
    viewer.setPosition(0,0);
    viewer.setCameraPosition(0.5,2,2,0.5,0,0,0,0,1);
    viewer.setBackgroundColor (0.2,0.2,0.2);
    viewer.addCoordinateSystem(1);
    utils::addVoxel(viewer, voxel, 0.005, "voxel", v1, 0, true );
    utils::addVoxel(viewer, voxel_gt, 0.005, "voxel_Gt", v2, 0, true );
    utils::addVoxel(viewer, eval, 0.005, "voxel_eval", v3, 0, true );
    utils::addVoxel(viewer, voxel_diff, 0.005, "voxel_diff", v4, 0, true );    
    viewer.spin();
#endif
}