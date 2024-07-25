#include <iostream>
#include <fstream>
#include <sstream>
#include <cctype>
#include <algorithm>
#include <vector>

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
  
  std::unordered_map<std::string, std::any> params;
  std::unordered_map<std::string, std::vector<std::any>> param_array;
  fetch_common_param(params, param_array);   // fetch parameter with key value

  if (params.size() != 0) {
    for(const auto& [key, value] : params) {
      std::cout << key << ": ";
      print_any(std::cout, value);
      std::cout << std::endl;
    }
  }

  if (param_array.size() != 0) {
    for(const auto& [key, values] : param_array) {
      std::cout << key << ": ";
      for (size_t i=0; i<values.size(); i++) {
        const auto & value = values[i];
        print_any(std::cout, value);
        if (i != values.size()-1) {
          std::cout << ", ";
        }
      }
      std::cout << std::endl;
    }
  }

  if (save_install) {
    for (auto const & dir : directories) {
      std::ifstream file(dir);  // open file
      // std::cout << "opening in " << dir.c_str() << std::endl;
      if (!file.is_open()) {  // check if file opened successfully
        std::cerr << "opening file failed" << std::endl;
        return false;
      }

      std::string line;
      std::string str = "odom_topic";
      int pos = 0;
      while (std::getline(file, line)) {
        if (line.find(str, pos) != std::string::npos) {
          // std::cout << "found!" << std::endl;
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

void Modifier::fetch_common_param(std::unordered_map<std::string, std::any> &result,
  std::unordered_map<std::string, std::vector<std::any>> &result_array,
  const std::string folder, 
  const std::string file)
{
  fs::path package_share_directory = ament_index_cpp::get_package_share_directory("node_parameter_handler");
  package_share_directory /= folder;
  package_share_directory /= file;
  if (!fs::exists(package_share_directory)) {
    std::cerr << "File does not exists!" << std::endl;
    return;
  }

  std::ifstream param_file(package_share_directory.native());  // open file
  if (!param_file.is_open()) {  // check if file opened successfully
    std::cerr << "opening file failed" << std::endl;
    return;
  }

  std::string line;
  char delimiter = ':';
  while (std::getline(param_file, line)) {
    line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());  // erase spacings
    std::vector<std::string> key_values = split_with_delimiter(line, delimiter);  // split with char
    if (key_values.size() == 2) {
      // array type must be wrapped with '[]'
      if (key_values[1].front()=='[' && key_values[1].back()==']') {
        std::vector<std::any> arrays;
        get_values_in_array(key_values[1], arrays);
        result_array.insert(std::make_pair(key_values[0], arrays));
      } else {
        result.insert(std::make_pair(key_values[0], get_value(key_values[1])));
      }
    }
  }
  return;
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

void Modifier::get_values_in_array(const std::string &string_values, std::vector<std::any> &final_result)
{
  std::string clean_string_array_values = string_values.substr(1, string_values.size() - 2);
  final_result = split_array(clean_string_array_values);
}

std::any Modifier::get_value(const std::string &word) const
{
  char sign='-', point='.';
  bool is_num = false, is_double = false;
  
  // decide value type
  if (word[0] == sign) {
    is_num = true;
  }
  for(const char &ch : word) {
    if (ch == point) {
      is_double = true;
      is_num = true;
      break;
    }
  }

  if (is_num) {
    if (is_double) {
      return std::stod(word);
    } else {
      return std::stoi(word);
    }
  } else {
    return word;
  }
}

std::vector<std::any> Modifier::split_array(const std::string &string_values) const
{
  char deli = ',';
  std::vector<std::string> result = split_with_delimiter(string_values, deli);

  if (result.size() == 0) {
    return std::vector<std::any>();
  }

  std::vector<std::any> final_array;
  final_array.reserve(result.size());
  try {
    for (auto const &r : result) {
      final_array.push_back(get_value(r));
    }
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
  }
  
  return final_array;
}

std::vector<std::string> Modifier::split_with_delimiter(const std::string &str, const char &delimiter) const
{
  std::istringstream iss(str);  // istringstream to hold the input string
  std::string buffer;           // buffer to hold each split part
  std::vector<std::string> result;  // vector to hold the result

  // Using getline to split the string by the delimiter
  while (std::getline(iss, buffer, delimiter)) {
    result.push_back(buffer);  // Add each part to the result vector
  }

  return result;
}
