#include <iostream>
#include "utils/utils_filesystem.hpp"

using namespace std;
namespace fs = boost::filesystem;

namespace utils
{

void ListFiles(const fs::path &root, vector<string> &filepaths)
{
    std::vector< std::string > all_matching_files;
    fs::directory_iterator end_itr; // Default ctor yields past-the-end
    for( fs::directory_iterator it( root ); it != end_itr; ++it )
    {
        if( fs::is_directory( it->status() ) )
        {
            ListFiles(it->path(), filepaths);
        }
        else
        {
            filepaths.push_back(it->path().string());
        }
    }
}

void ListFiles(const fs::path &root, vector<fs::path> &filepaths)
{
    std::vector< std::string > all_matching_files;
    fs::directory_iterator end_itr; // Default ctor yields past-the-end
    for( fs::directory_iterator it( root ); it != end_itr; ++it )
    {
        if( fs::is_directory( it->status() ) )
        {
            ListFiles(it->path(), filepaths);
        }
        else
        {
            filepaths.push_back(it->path());
        }
    }
}

static bool compare_files(fs::path f1, fs::path f2)
{
    return f1.string() < f2.string();
}

void ListFiles(const string &root, vector<std::string> &filepaths)
{
    fs::path path_root(root);
    ListFiles(path_root, filepaths);
    std::sort (filepaths.begin(), filepaths.end(), compare_files);
}

void ListFiles(const string &root, vector<fs::path> &filepaths)
{
    fs::path path_root(root);
    ListFiles(path_root, filepaths);
    std::sort (filepaths.begin(), filepaths.end(), compare_files);    
}

}
