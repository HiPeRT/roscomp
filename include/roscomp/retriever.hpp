#if TKROS_VERSION == 1
#include <ros/package.h>
#elif TKROS_VERSION == 2
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

namespace resource_retriever {

class Retriever
{
  public:
    std::string getPath(const std::string& url)
    {
        std::string mod_url = url;
        if (url.find("package://") == 0) {
            mod_url.erase(0, strlen("package://"));
            size_t pos = mod_url.find("/");
            if (pos == std::string::npos) {
                throw std::runtime_error(
                  "Error retrieving file [" + url +
                  "]: Could not parse package:// format into file:// format");
            }

            std::string package = mod_url.substr(0, pos);
            mod_url.erase(0, pos);
            std::string package_path;

#if TKROS_VERSION == 1
            package_path = ::ros::package::getPath(package);
            if (package_path.empty()) {
                throw std::runtime_error("Error retrieving file [" + url + "]: Package [" +
                                         package + "] does not exist");
            }
#elif TKROS_VERSION == 2
            try {
                package_path = ament_index_cpp::get_package_share_directory(package);
            } catch (const ament_index_cpp::PackageNotFoundError& error) {
                throw std::runtime_error("Error retrieving file [" + url + "]: Package [" +
                                         package + "] does not exist");
            }
#endif
            return package_path + mod_url;
        }
        return "";
    }
};

} // namespace resource_retriever