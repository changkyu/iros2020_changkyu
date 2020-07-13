#include <string>
#include <vector>
#include <boost/filesystem.hpp>

namespace utils
{

void ListFiles( const std::string &root, 
                std::vector<std::string> &filepaths );
void ListFiles( const std::string &root, 
                std::vector<boost::filesystem::path> &filepaths );

}
