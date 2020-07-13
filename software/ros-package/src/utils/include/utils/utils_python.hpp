#ifndef _UTILS_PYTHON__HPP_
#define _UTILS_PYTHON__HPP_

#include <vector>
#include "utils/utils.hpp"

namespace utils
{

void SaveTSDF2Numpy( const char* filepath, tsdf_t &tsdf );
void LoadTSDF2Numpy( const char* filepath, tsdf_t &tsdf, const std::string &load_funct="load" );
void SaveVoxel2Numpy(const char* filepath, const voxel_t &voxel);
void LoadVoxel2Numpy(const char* filepath, voxel_t &voxel);
void SaveBoolVoxel2Mat(const char* filepath, voxel_t &voxel, const char* name);
void LoadBoolVoxelfromMat(const char* filepath, voxel_t &voxel, const char* name);
}

#endif