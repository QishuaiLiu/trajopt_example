#ifndef TRAJOPT_UTILS_
#define TRAJOPT_UTILS_

#include <iostream>
#include <boost/asio/ip/host_name.hpp>                        // for hostname
#include <boost/interprocess/detail/os_thread_functions.hpp>  // for process / thread IDs
#include <boost/filesystem.hpp>                               // for filesystem paths
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // for UUID generation
#include <boost/uuid/uuid_generators.hpp>  // for UUID generation
#include <boost/uuid/uuid_io.hpp>          // for UUID generation

#include <ros/package.h>  // for package resolving
#include <ros/ros.h>
#include <yaml.h>
#include <yaml-cpp/yaml.h>  // for YAML parsing
#include <moveit/planning_scene/planning_scene.h>

using RobotPose = std::decay<                                      //
    decltype(                                                      //
        std::declval<moveit::core::Transforms>().getTransform("")  //
        )                                                          //
    >::type;

namespace scene
{
bool isPrefix(const std::string& lhs, const std::string& rhs);

bool isSuffix(const std::string& lhs, const std::string& rhs);

boost::filesystem::path expandHome(const boost::filesystem::path& in);

boost::filesystem::path expandSymlinks(const boost::filesystem::path& in);

boost::filesystem::path expandPath(const boost::filesystem::path& in);

std::string resolvePackage(const std::string& path);

std::string resolvePath(const std::string& path);

bool isExtension(const std::string& path_string, const std::string& extension);

std::pair<bool, YAML::Node> loadFileToYAML(const std::string& path);

template <typename T>
bool YAMLFileToMessage(T& msg, const std::string& file)
{
  const auto& result = loadFileToYAML(file);
  if (result.first)
    msg = result.second.as<T>();

  return result.first;
}

int testFunc(int x);
};  // namespace scene
#endif
