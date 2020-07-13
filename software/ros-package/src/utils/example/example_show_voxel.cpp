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

int main(int argc, char* argv[])
{   
    string fp_input;
    string type;    
    po::options_description desc("Example Usage");
    desc.add_options()
        ("help", "help")
        ("input,i",  po::value<string>(&fp_input), "input file")
        ("type,t",   po::value<string>(&type),     "type (tsdf,voxel)"      )        
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if( vm.count("help")        ||        
        fp_input.compare("")==0 ||
        type.compare("")==0        )
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

    if( type.compare("tsdf")  && 
        type.compare("voxel")    )
    {
        cerr << "[Error] Invalid type: " << type << endl;
        return 0;
    }

    visualization::PCLVisualizer viewer;
    viewer.setWindowName("debug");
    viewer.setSize(500,500);
    viewer.setPosition(0,0);
    viewer.setCameraPosition(0.5,2,2,0.5,0,0,0,0,1);
    viewer.setBackgroundColor (0.2,0.2,0.2);
    viewer.addCoordinateSystem(1);

    const float resolution = 0.005;
    if( type.compare("tsdf")==0 )
    {
        tsdf_t tsdf;
        LoadTSDF2Numpy(fp_input.c_str(), tsdf);
       
        addTSDF(viewer, tsdf, 0.005);
    }
    else if( type.compare("voxel")==0 )
    {
        voxel_t voxel;
        LoadVoxel2Numpy(fp_input.c_str(),voxel );

        addVoxel(viewer, voxel, 0.005);
    }

    viewer.spin();
}