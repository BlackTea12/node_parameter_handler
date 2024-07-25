#ifndef FILE_READER_CLASS_HPP_
#define FILE_READER_CLASS_HPP_

#include <filesystem>
#include <string>
#include <vector>
#include <any>
#include <unordered_map>

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
   * @brief Get common parameter
   * @param folder common.yaml placed folder
   * @param file file name which will be 'common.yaml'
   */
  void fetch_common_param(std::unordered_map<std::string, std::any> &result, 
    std::unordered_map<std::string, std::vector<std::any>> &result_array,
    const std::string folder="params", 
    const std::string file="common.yaml");

  /**
   * @brief get all file directories to check
   * @return file directories
   */
  std::vector<std::string> get_file_directories(const fs::path &root, const bool& search_deep);

  /**
   * @brief Get values from string
   * @param string_values
   * @param final_result
   * @return value configured
   */
  void get_values_in_array(const std::string &string_values, std::vector<std::any> &final_result);

  /**
   * @brief Get values from string
   * @param string_values
   * @param final_result
   * @return value configured
   */
  void get_values(const std::string &string_values, std::any &final_result);

  /**
   * @brief get value from word
   * @param word one seperate value
   * @return fitted data type
   */
  std::any get_value(const std::string &word) const;
  /**
   * @brief Check if string is number
   * @return vector of any
   */
  std::vector<std::any> split_array(const std::string &string_values) const;

  /**
   * @brief split string with desired delimiter
   */
  std::vector<std::string> split_with_delimiter(const std::string &str, const char &delimiter) const;

  /**
   * @brief print any type value with typeid
   */
  template<typename T>
  void print_value(std::ostream& os, const std::any& value) {
    if (value.type() == typeid(T)) {
      os << std::any_cast<T>(value);
    }
  }

  /**
   * @brief print any type value
   */
  void print_any(std::ostream& os, const std::any& value) {
    if (value.type() == typeid(int)) {
      print_value<int>(os, value);
    } else if (value.type() == typeid(double)) {
      print_value<double>(os, value);
    } else if (value.type() == typeid(std::string)) {
      print_value<std::string>(os, value);
    } else if (value.type() == typeid(const char*)) {
      print_value<const char*>(os, value);
    } else {
      os << "Unknown type, int, double, string, char*";
    }
  }
  
  ///@brief root package pointer
  std::string* root_pkg_ptr_;
  ///@brief root parameter folder pointer
  std::string* root_param_folder_ptr_;
};

#endif  // FILE_READER_CLASS_HPP_
