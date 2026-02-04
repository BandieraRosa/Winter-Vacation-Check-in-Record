# 2.4

最近正在为我的机器人上位机开发基于 ROS2 的通用相机功能包，通过 ROS2 Plugin 机制动态加载不同的派生相机类包以实现多相机型号适配。目前已经完成基类与相机节点的编写，并且完成了海康相机与华睿相机的对应驱动包编写。

然而有一个比较突出的问题是，我的插件匹配实现很简陋，依赖于在相机节点中配置对应的分支控制。

```c++
// camera_node.cpp
bool CameraNode::LoadCameraPlugin()
{
  try
  {
    camera_loader_ = std::make_unique<pluginlib::ClassLoader<CameraBase>>(
        "camera_node", "Camera::CameraBase");

    // 开始匹配插件包
    std::string plugin_name;
    if (camera_type_ == "hik_camera" || camera_type_ == "hik")
    {
      plugin_name = "HikCamera::HikCamera";
    }
    else if (camera_type_ == "huaray_camera" || camera_type_ == "huaray")
    {
      plugin_name = "HuarayCamera::HuarayCamera";
    }
    else
    {
      plugin_name = camera_type_;
    }
    //

    RCLCPP_INFO(this->get_logger(), "Loading camera plugin: %s", plugin_name.c_str());

    camera_ = camera_loader_->createSharedInstance(plugin_name);

    if (!camera_)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create camera instance");
      return false;
    }

    camera_->SetParams(params_);

    return true;
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load camera plugin: %s", ex.what());
    return false;
  }
}
```

这就导致我每次添加新的相机型号适配都必须重新编译，非常地不方便。

阅读 `ClassLoader` 源码时，发现该函数：

```c++
// class_loader_imp.hpp
template<class T>
std::vector<std::string> ClassLoader<T>::getDeclaredClasses()
/***************************************************************************/
{
  std::vector<std::string> lookup_names;
  for (ClassMapIterator it = classes_available_.begin(); it != classes_available_.end(); ++it) {
    lookup_names.push_back(it->first);
  }

  return lookup_names;
}
```

可以获取所有已加载的插件类名。基于此，我可以动态地添加相机驱动，只要保持一定的命名规范。

于是我约定之后的相机插件类命名应当如下：

```c++
package_name = vendor_camera;
full_class_declare = VendorCamera::VendorCamera;
```

接下来实现了通过插件信息来匹配相机型号。

```c++
// camera_node.cpp
bool CameraNode::LoadCameraPlugin()
{
  try
  {
    camera_loader_ = std::make_unique<pluginlib::ClassLoader<CameraBase>>(
        "camera_node", "Camera::CameraBase");

    auto available_plugins = camera_loader_->getDeclaredClasses();

    RCLCPP_INFO(this->get_logger(), "Available camera plugins:");
    for (const auto& plugin : available_plugins)
    {
      RCLCPP_INFO(this->get_logger(), "  - %s", plugin.c_str());
    }

    std::string plugin_name = FindMatchingPlugin(camera_type_, available_plugins);

    if (plugin_name.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "No matching plugin found for camera_type '%s'",
                   camera_type_.c_str());
      RCLCPP_ERROR(this->get_logger(), "Available plugins: %s",
                   FormatPluginList(available_plugins).c_str());
      RCLCPP_ERROR(this->get_logger(), "Usage: camera_type can be:");
      RCLCPP_ERROR(this->get_logger(),
                   "  1. Package name (e.g., 'hik_camera', 'huaray_camera')");
      RCLCPP_ERROR(this->get_logger(), "  2. Vendor name (e.g., 'hik', 'huaray')");
      RCLCPP_ERROR(this->get_logger(),
                   "  3. Full class name (e.g., 'HikCamera::HikCamera')");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Loading camera plugin: %s", plugin_name.c_str());

    // 加载插件
    camera_ = camera_loader_->createSharedInstance(plugin_name);

    if (!camera_)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create camera instance");
      return false;
    }

    // 设置 node 指针（pluginlib 使用默认构造函数，需要手动设置）
    camera_->SetNode(this);

    // 设置参数
    camera_->SetParams(params_);

    return true;
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load camera plugin: %s", ex.what());
    return false;
  }
}

std::string CameraNode::FindMatchingPlugin(
    const std::string& camera_type, const std::vector<std::string>& available_plugins)
{
  std::string camera_type_lower = camera_type;
  std::transform(camera_type_lower.begin(), camera_type_lower.end(),
                 camera_type_lower.begin(), ::tolower);

  for (const auto& plugin : available_plugins)
  {
    std::string plugin_lower = plugin;
    std::transform(plugin_lower.begin(), plugin_lower.end(), plugin_lower.begin(),
                   ::tolower);

    // 直接匹配完整类名
    if (plugin_lower == camera_type_lower)
    {
      RCLCPP_INFO(this->get_logger(), "Matched by full class name: %s", plugin.c_str());
      return plugin;
    }

    // 匹配包名
    if (camera_type_lower.find("_camera") != std::string::npos)
    {
      std::string vendor = camera_type_lower.substr(0, camera_type_lower.find("_camera"));
      std::string expected_plugin_lower = vendor + "camera::" + vendor + "camera";
      if (plugin_lower == expected_plugin_lower)
      {
        RCLCPP_INFO(this->get_logger(), "Matched by package name: %s", plugin.c_str());
        return plugin;
      }
    }

    // 匹配厂商名
    if (camera_type_lower.length() < 8)
    {
      std::string expected_plugin_lower =
          camera_type_lower + "camera::" + camera_type_lower + "camera";
      if (plugin_lower == expected_plugin_lower)
      {
        RCLCPP_INFO(this->get_logger(), "Matched by vendor name: %s", plugin.c_str());
        return plugin;
      }
    }

    // 部分匹配
    if (plugin_lower.find(camera_type_lower) != std::string::npos)
    {
      RCLCPP_INFO(this->get_logger(), "Matched by partial match: %s", plugin.c_str());
      return plugin;
    }
  }

  return "";
}
```

然而还有问题。在 ROS2 文档中提到：

> The code above creates an abstract class called `RegularPolygon`. One thing to notice is the presence of the initialize method. With `pluginlib`, a constructor without parameters is required, so if any parameters to the class are needed, we use the initialize method to pass them to the object.

插件加载使用默认构造函数。我的相机类需要传入 `rclcpp::Node` 指针用于对齐时间戳，所以我还需要一个额外的工具函数用于初始化参数。

```c++
// camera_base
void SetNode(rclcpp::Node* node) { node_ = node; }
```

修改完成，编译：

```bash
bandiera@BR:~/Desktop/camera_packages$ colcon build --symlink-install
Starting >>> camera_node
Finished <<< camera_node [0.07s]
Starting >>> hik_camera
Starting >>> huaray_camera
Finished <<< hik_camera [0.06s]                                                         
Finished <<< huaray_camera [0.07s]

Summary: 3 packages finished [0.30s]
```

启动节点：

```bash
bandiera@BR:~/Desktop/camera_packages$ ros2 launch camera_node camera_launch.py 
[INFO] [launch]: All log files can be found below /home/bandiera/.ros/log/2026-02-04-21-06-44-026938-BR-86311
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [camera_node_node-1]: process started with pid [86312]
[camera_node_node-1] [INFO] [1770210404.066687800] [camera_node]: Parameters have been initialized.
[camera_node_node-1] [INFO] [1770210404.067068832] [camera_node]: Available camera plugins:
[camera_node_node-1] [INFO] [1770210404.067086346] [camera_node]:   - HikCamera::HikCamera
[camera_node_node-1] [INFO] [1770210404.067095047] [camera_node]:   - HuarayCamera::HuarayCamera
[camera_node_node-1] [INFO] [1770210404.067116248] [camera_node]: Matched by package name: HikCamera::HikCamera
[camera_node_node-1] [INFO] [1770210404.067121707] [camera_node]: Loading camera plugin: HikCamera::HikCamera
[camera_node_node-1] [INFO] [1770210404.068927426] [camera_node]: Camera plugin loaded: hik_camera
[camera_node_node-1] [INFO] [1770210404.103521410] [camera_node]: Camera publisher created.
[camera_node_node-1] [INFO] [1770210404.331781518] [camera_node]: Starting to grab...
[camera_node_node-1] [INFO] [1770210404.845486327] [camera_node]: Hik camera initialized and started.
[camera_node_node-1] [INFO] [1770210404.845536180] [camera_node]: Camera initialized.
[camera_node_node-1] [INFO] [1770210404.845643918] [camera_node]: Guard thread created.
[camera_node_node-1] [INFO] [1770210404.845757394] [camera_node]: Protect thread started.
[camera_node_node-1] [INFO] [1770210404.846050101] [camera_node]: camera calibration URL: package://camera_node/config/camera_info.yaml
[camera_node_node-1] [INFO] [1770210404.846473074] [camera_node]: Capture thread started.
```

无报错，提交。

[仓库链接]: https://github.com/BandieraRosa/camera
[提交链接]: https://github.com/BandieraRosa/camera/commit/574f7c89545b19bd65776102debdbb244ac9f6ea
[个人主页]: https://github.com/BandieraRosa

