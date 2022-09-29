
### 测试
```bash
NiViewer 
```

### 下载OpenNI
```bash
wget http://dl.orbbec3d.com/dist/openni2/OpenNI_2.3.0.55.zip
```
[SDK官方网站](https://orbbec3d.com/develop/#linux)
解压，选择OpenNI-Linux-x64-2.3.0.55这个文件，单独拷贝出来


### 安装依赖
```bash
sudo apt-get install build-essential freeglut3 freeglut3-dev 
sudo apt-get install ros-kinetic-astra-camera
sudo apt-get install libgl1-mesa-dri
```
查看udev的版本，奥比中光的驱动依赖libudev.so.1，看看有没有关联在一起：
```bash
ldconfig -p | grep libudev.so.1
```

### 安装及编译
```bash
chmod +x install.sh
sudo ./install.sh
```
添加环境变量
```bash
source OpenNIDevEnvironment

cd Samples/SimpleViewer
make 
cd Bin/x64-Release
./SimpleViewer


cd ../../../.. && cd Samples/ClosestPointViewer
make 
cd Bin/x64-Release
./ClosestPointViewer

cd ../../../.. &&  cd Samples/EventBasedRead
make

cd ../.. &&  cd Samples/MultiDepthViewer
make

cd ../.. &&  cd Samples/MultipleStreamRead
make

cd ../.. &&  cd Samples/MWClosestPoint
make

cd ../.. &&  cd Samples/MWClosestPointApp
make

cd ../.. &&  cd Samples/SimpleRead
make

```
下载astra.launch并编译
```bash



```
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
找不到彩色图像


### 安装彩色驱动
编译安装libuvc
```bash
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build 
cmake ..
make && sudo make install
```

安装libuvc_ros
```bash
sudo apt-get install ros-kinetic-libuvc-ros
```
测试：
```bash
roscore

rosrun libuvc_camera camera_node
```

如果出现下面的情况：
```xml
[ERROR] [1567993406.870407681]: Permission denied opening /dev/bus/usb/001/003
```

修改端口权限：
```bash
sudo chmod 777 /dev/bus/usb/001/003
```
出现下面问题：
```bash
[ INFO] [1567997483.760670632]: Opening camera with vendor=0x0, product=0x0, serial="", index=0
unsupported descriptor subtype VS_COLORFORMAT
uvc_get_stream_ctrl_format_size: Invalid mode (-51)
[ERROR] [1567997484.014215308]: check video_mode/width/height/frame_rate are available
```



