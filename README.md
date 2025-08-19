# Minimal Grasp Environment (最小化抓取环境)

本项目旨在提供一个尽可能简化的 Franka Panda 抓取环境，方便快速复现 **ROS + Gazebo + MoveIt** 下的抓取实验。

---

## 构建步骤

### 1. 编译工程
在工作空间下执行：
```bash
catkin_make
```
在编译过程中，可能会因为缺少依赖而报错。
此时可以根据报错信息，利用 AI 或搜索引擎 来提示需要安装的依赖，例如：
```bash
sudo apt install ros-melodic-<package-name>
```
逐步补齐所有依赖后即可完成构建。

2. 配置环境变量
在 ~/.bashrc 中添加以下行，使 Gazebo 能找到自定义模型：

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/zlwq/minimal_grasp/src/panda_demo/urdf
```
保存后刷新：

```bash
source ~/.bashrc
```
3. 加载工作空间环境
构建完成后，在 ~/.bashrc 或手动执行：

```bash
source /home/zlwq/franka_ros/devel/setup.bash
```
使用说明
完成以上步骤后，即可在 Gazebo 中启动 最小化 Panda 抓取环境，并基于此进行抓取复现。

注意事项
本配置为 最小化抓取环境，因此许多非必要功能和插件已被精简。

如需额外功能，请在编译过程中根据报错提示自行安装缺失的 ROS 依赖包。

致谢
本环境基于 franka_ros和官方教程moviet_tutorial 改造而来，感谢原作者的开源贡献。
