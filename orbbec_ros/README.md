# orbbec_ros

### 安装依赖
```bash
sudo apt-get install build-essential freeglut3 freeglut3-dev 
sudo apt-get install ros-kinetic-astra-camera
sudo apt-get install libgl1-mesa-dri
sudo apt-get install ros-kinetic-libuvc-ros
#sudo apt-get install ros-kinetic-uvc-camera 
sudo apt-get install v4l-utils
```

### 安装驱动
```bash
cd ~/catkin_ws/src
git clone https://github.com/GJXS1980/orbbec_ros.git
cd orbbec_ros/camera_driver
chmod +x install.sh
sudo ./install.sh

# 编译
cd ~/catkin_ws
catkin_make
```


### 测试
通过
```bash
ls /dev/video*
```
查看相机所在的端口并修改astra.launch文件。


测试：
```bash
roslaunch astra_launch astra.launch

rqt_image_view
```
订阅/image_raw话题


如果出现下面的情况：
```xml
[ERROR] [1567993406.870407681]: Permission denied opening /dev/bus/usb/001/003
```


修改端口权限：
```bash
sudo chmod 777 /dev/bus/usb/001/003
```



