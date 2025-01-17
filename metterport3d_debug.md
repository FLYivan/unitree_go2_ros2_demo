由于文档内容较长，我将分段为您翻译，并以Markdown格式化输出。

### Matterport 3D环境设置说明

#### 设置仓库
首先，请按照我们网站上的说明设置`autonomous_exploration_development_environment`仓库，并使用默认设置。然后，检出`distribution-matterport`分支（根据ROS版本，将`distribution`替换为`melodic`或`noetic`），并进行编译。

```bash
cd autonomous_exploration_development_environment
git checkout distribution-matterport && catkin_make
```

#### 准备环境模型
安装MeshLab以进行网格格式转换。

MeshLab: [https://www.meshlab.net](https://www.meshlab.net)

前往Matterport 3D网站，签署使用条款，并使用提供的`download_mp.py`脚本下载环境模型。

Matterport 3D: [https://niessner.github.io/Matterport](https://niessner.github.io/Matterport)

在每个环境文件夹中，您会找到一个`matterport_mesh.zip`文件。以环境ID:17DRP 5sb 8fy（首次下载的网格）为例，解压`matterport_mesh.zip`文件，并在MeshLab中加载`.obj`文件，您将看到一个彩色网格。将网格导出为DAE格式，并保存为`matterport.dae`文件，与所有其他网格文件（即`.mtl`、`.obj`、`.jpg`）保存在同一文件夹中。然后，将所有网格文件（即`.mtl`、`.obj`、`.jpg`）以及`matterport.dae`文件复制到`src/vehicle_simulator/mesh/matterport/meshes`文件夹中（不要将文件放在`matterport`文件夹内的另一个文件夹中）。

您还可以找到一个`house_segmentations.zip`文件。解压`.house`文件，将其重命名为`matterport.house`文件，并复制到`src/vehicle_simulator/mesh/matterport/segmentations`文件夹中。解压`.ply`文件，将其重命名为`pointcloud.ply`文件，并复制到`src/vehicle_simulator/mesh/matterport/preview`文件夹中。

#### 启动系统
在终端中，加载ROS工作空间并启动系统。

```bash
source devel/setup.sh
roslaunch vehicle_simulator system_matterport.launch
```

现在，用户可以在RVIZ中使用“Waypoint”按钮来驾驶车辆。渲染的RGB图像（左图）、深度图像（中图）和注册扫描应显示在RVIZ中。要查看与深度图像对应的点云消息（右图），在RVIZ中点击“Panels->Displays”并勾选“depthCloud”。要查看整体地图（下图）、已探索区域和车辆轨迹，请勾选“overallMap”、“exploredAreas”和“trajectory”。相机配置设置在`src/vehicle_simulator/urdf/rgbd_camera.urdf.xacro`文件中。

要查看区域分割（上图）和对象分割（下图），在RVIZ中点击“Panels->Displays”并勾选“regionMarkers”和“objectMarkers”。分割数据也作为`sensor_msgs::PointCloud2`类型的消息在ROSTopics `/region_segmentations`和`/object_segmentations`上发布。点定义位于`src/segmentation_proc/src/segmentationProc.cpp`文件中。区域标签和对象类别保留了Matterport 3D的原始定义。

在第二个终端中，进入`autonomous_exploration_development_environment`文件夹，加载ROS工作空间，然后使用以下命令运行一个航点示例。车辆将遵循航点并在导航边界内行驶（航点和导航边界是专门为环境ID:17DRP 5sb 8fy设计的）。

```bash
roslaunch waypoint_example waypoint_example_matterport.launch
```

要查看Gazebo GUI中的网格和车辆，请在`src/vehicle_simulator/launch/system_matterport.launch`中设置`gazebo_gui = true`。要配置车辆前进和后退，请在`src/local_planner/launch/local_planer.launch`中设置`twoWayDrive = true`。要从不同的位置或方向启动车辆，请在`src/vehicle_simulator/launch/system_matterport.launch`中设置`vehicleX`、`vehicleY`、`terrainZ`、`vehicleYaw`。

#### 选择起始位置
安装CloudCompare以进行点云查看和编辑。

CloudCompare: [https://www.danielgm.net/cc](https://www.danielgm.net/cc)

Matterport 3D环境模型通常有多个楼层。我们建议使用CloudCompare在所需楼层上选择车辆起始位置。加载从`house_segmentations.zip`文件中提取的`.ply`文件到CloudCompare中。然后，您可以使用横截面工具裁剪掉天花板和未使用的楼层，以显示起始区域。

使用点选择工具选择车辆起始位置。您可以使用这些值来设置`vehicleX`、`vehicleY`、`terrainZ`在`src/vehicle_simulator/launch/system_matterport.launch`中。您还可以将裁剪后的点云保存为`pointcloud.ply`文件，并替换`src/vehicle_simulator/mesh/matterport/preview`文件夹中的文件。

#### 使用Habitat进行离线处理
安装Habitat进行图像渲染（首先安装Anaconda）。

Habitat: [https://github.com/facebookresearch/habitat-sim](https://github.com/facebookresearch/habitat-sim)

Anaconda: [https://www.anaconda.com/products/individual](https://www.anaconda.com/products/individual)

推荐使用Conda包安装Habitat，需要Python>=3.6和cmake>=3.10。安装Conda后，使用以下命令行安装Habitat。

```bash
conda create -n habitat python=3.6 cmake=3.14.0
conda activate habitat
```

对于带有显示器的计算机，请使用：

```bash
conda install habitat-sim -c conda-forge -c aihabitat
```

如果您想安装特定版本，例如v 0.1.7而不是最新稳定版本，请使用：

```bash
conda install habitat-sim=0.1.7 -c conda-forge -c aihabitat
```

然后，使用带有`--task habitat`标志的Matterport 3D脚本来下载环境模型，需要Python 2.7。这将下载一个为Habitat准备的`mp3d_habitat.zip`文件。使用与系统相同的环境ID: 17DRP 5sb 8fy提取文件。

```bash
python 2 download_mp.py --task habitat -o data_download_dir
```

在终端中，克隆Habitat-sim仓库的稳定分支，并运行示例脚本进行测试。如果图像正确保存，意味着Habitat安装正确。

```bash
git clone --branch stable https://github.com/facebookresearch/habitat-sim.git
cd habitat-sim/examples
python 3 example.py --scene extracted_mp3d_habitat_dir/environment_id.glb \
--semantic_sensor --depth_sensor --save_png
```

我们的系统会在`src/vehicle_simulator/log`文件夹中为每次运行记录一个`trajectory_timestamp.txt`文件。使用文本编辑器查看`trajectory_timestamp.txt`文件，其中每一行是一个姿态，包括x (m), y (m), z (m), roll (rad), pitch (rad), yaw (rad), 从开始的时间（秒）。定位您想要后处理的运行的`trajectory_timestamp.txt`文件。在终端中，进入`src/segmentation_proc/scripts`文件夹，并运行命令行。

```bash
python 3 habitat_offline_v0.x.x.py --scene extracted_mp3d_habitat_dir/environment_id.glb \
--trajectory trajectory_file_dir/trajectory_timestamp.txt --save_dir image_saving_dir
```

这将在`image_saving_dir`文件夹中渲染并保存RGB图像、深度图像和语义图像，如命令行中指定。图像将为`trajectory_timestamp.txt`文件中的每个姿态进行渲染。

请注意，系统需要在原生Ubuntu上启动，并且`habitat_offline_v0.x.x.py`脚本需要在conda-habitat环境中运行。在启动系统之前，请使用以下命令退出conda-habitat环境。

```bash
conda deactivate
```

在运行`habitat_offline_v0.x.x.py`脚本之前，请使用此命令行进入conda-habitat环境。

```bash
conda activate habitat
```

#### 使用Habitat进行在线处理（仅限Ubuntu 20.04）
在Ubuntu 20.04计算机上，可以在conda-habitat环境中与系统并行运行Habitat。除了在原生Ubuntu上的标准ROS Noetic安装外，还需要在conda-habitat环境中安装ROS Noetic。

```bash
conda activate habitat
conda install -c conda-forge -c robostack ros-noetic-desktop
```

在终端中启动系统。在第二个终端中，进入`src/segmentation_proc/scripts`文件夹，并运行`habitat_online_v0.x.x.py`脚本（仅在`noetic-matterport`分支上可用）。

```bash
conda activate habitat
python 3 habitat_online_v0.x.x.py --scene extracted_mp3d_habitat_dir/environment_id.glb
```

现在，用户可以在RVIZ