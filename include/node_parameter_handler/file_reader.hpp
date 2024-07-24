#ifndef FILE_READER_CLASS_HPP_
#define FILE_READER_CLASS_HPP_

#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

class Modifier
{
public:
  /**
   * @brief Constructor
   * @param pkg_name package name
   * @param param_folder_name parameter folder name
   */
  Modifier(
    std::string* pkg_name, 
    std::string* param_folder_name);

  /**
   * @brief Destructor
   */
  ~Modifier();

  /**
   * @brief start modify
   * @return true when successful, false if not
   */
  bool start(const bool& search_deep, const bool& save_install, const bool& save_home);

protected:
  /**
   * @brief get all file directories to check
   * @return file directories
   */
  std::vector<std::string> get_file_directories(const fs::path &root, const bool& search_deep);

  ///@brief root package pointer
  std::string* root_pkg_ptr_;
  ///@brief root parameter folder pointer
  std::string* root_param_folder_ptr_;
};

#endif  // FILE_READER_CLASS_HPP_
