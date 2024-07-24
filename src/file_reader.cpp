#include <iostream>
#include <fstream>

#include "node_parameter_handler/file_reader.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

Modifier::Modifier(std::string *pkg_name, std::string *param_folder_name)
{
  root_pkg_ptr_ = pkg_name;
  root_param_folder_ptr_ = param_folder_name;
}

Modifier::~Modifier()
{
}

bool Modifier::start(const bool &search_deep, const bool &save_install, const bool &save_home)
{
  std::string package_share_directory = ament_index_cpp::get_package_share_directory(*root_pkg_ptr_);
  package_share_directory += "/"+*root_param_folder_ptr_;
  std::vector<std::string> directories = get_file_directories(package_share_directory, search_deep);

  if (save_install) {
    for (auto const & dir : directories) {
      std::ifstream file(dir);  // open file
      std::cout << "opening in " << dir.c_str() << std::endl;
      if (!file.is_open()) {  // check if file opened successfully
        std::cerr << "opening file failed" << std::endl;
        return false;
      }

      std::string line;
      std::string str = "odom_topic";
      int pos = 0;
      while (std::getline(file, line)) {
        if (line.find(str, pos) != std::string::npos) {
          std::cout << "found!" << std::endl;
        }
      }
    }
  }

  if (save_home) {
    fs::path home_path;
    int cnt = 0;
    for (const auto& part : fs::current_path()) {
      if (cnt < 3) {
        home_path /= part;
        ++cnt;
      } else {
        break;
      }
    }
    
    home_path /= "modify_yaml_result";  // default saving directory

    if (!fs::exists(home_path) || !fs::is_directory(home_path)) {
      fs::create_directories(home_path);
    }
  }

  return false;
}

std::vector<std::string> Modifier::get_file_directories(const fs::path &root, const bool& search_deep)
{
  if (!fs::exists(root) || !fs::is_directory(root)) {
    std::cerr << "Path is neither a directory nor exists!" << std::endl;
    return std::vector<std::string>();
  }

  std::vector<std::string> result;
  if (search_deep) {
    for (const auto& entry : fs::recursive_directory_iterator(root)) {
      if (fs::is_regular_file(entry)) {
        result.push_back(entry.path().string());
      }
    }
  } else {
    for (const auto& entry : fs::directory_iterator(root)) {
      if (fs::is_regular_file(entry)) {
        result.push_back(entry.path().string());
      }
    }
  }

  if (result.size() == 0) {
    std::cerr << "No parameters to modify!" << std::endl;
    return std::vector<std::string>();
  }

  return result;
}
