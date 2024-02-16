#include <utils.h>

namespace scene
{
bool isPrefix(const std::string& lhs, const std::string& rhs)
{
  return std::equal(lhs.begin(), lhs.begin() + std::min(lhs.size(), rhs.size()), rhs.begin());
}

bool isSuffix(const std::string& lhs, const std::string& rhs)
{
  return std::equal(lhs.rbegin(), lhs.rbegin() + std::min(lhs.size(), rhs.size()), rhs.rbegin());
}

boost::filesystem::path expandHome(const boost::filesystem::path& in)
{
  const char* home = std::getenv("HOME");
  if (home == nullptr)
  {
    ROS_WARN("HOME Environment variable is not set! Cannot resolve ~ in path.");
    return in;
  }

  boost::filesystem::path out;
  for (const auto& p : in)
    out /= (p.string() == "~") ? home : p;

  return out;
}

boost::filesystem::path expandSymlinks(const boost::filesystem::path& in)
{
  // Check if the path has a symlink before expansion to avoid error.
  boost::filesystem::path out;
  for (const auto& p : in)
  {
    auto tmp = out / p;
    if (boost::filesystem::is_symlink(tmp))
      return boost::filesystem::canonical(in);
  }

  return in;
}

boost::filesystem::path expandPath(const boost::filesystem::path& in)
{
  boost::filesystem::path out = in;
  out = expandHome(out);
  out = expandSymlinks(out);

  return boost::filesystem::absolute(out);
}

std::string resolvePackage(const std::string& path)
{
  if (path.empty())
    return "";

  const std::string prefix = "package://";

  boost::filesystem::path file;
  if (isPrefix(prefix, path))
  {
    boost::filesystem::path subpath(path.substr(prefix.length(), path.length() - 1));
    const std::string package_name = (*subpath.begin()).string();

    const std::string package = ros::package::getPath(package_name);
    if (package.empty())
    {
      ROS_WARN("Package `%s` does not exist.", package_name.c_str());
      return "";
    }

    file = package;
    for (auto it = ++subpath.begin(); it != subpath.end(); ++it)
      file /= *it;
  }
  else
    file = path;

  return expandPath(file).string();
}

std::string resolvePath(const std::string& path)
{
  boost::filesystem::path file = resolvePackage(path);

  if (!boost::filesystem::exists(file))
  {
    ROS_WARN("File `%s` does not exist.", path.c_str());
    return "";
  }

  return boost::filesystem::canonical(boost::filesystem::absolute(file)).string();
}

bool isExtension(const std::string& path_string, const std::string& extension)
{
  boost::filesystem::path path(path_string);
  const std::string last = boost::filesystem::extension(path);
  return isSuffix(extension, last);
}

std::pair<bool, YAML::Node> loadFileToYAML(const std::string& path)
{
  YAML::Node file;
  const std::string full_path = resolvePath(path);
  if (full_path.empty())
    return std::make_pair(false, file);

  if (!isExtension(full_path, "yml") && !isExtension(full_path, "yaml"))
    return std::make_pair(false, file);

  try
  {
    return std::make_pair(true, YAML::LoadFile(full_path));
  }
  catch (std::exception& e)
  {
    return std::make_pair(false, file);
  }
}

// template <typename T>
// bool YAMLFileToMessage(T& msg, const std::string& file)
// {
//   const auto& result = loadFileToYAML(file);
//   if (result.first)
//     msg = result.second.as<T>();

//   return result.first;
// }

int testFunc(int x)
{
  return x + 5;
}
}  // namespace scene
