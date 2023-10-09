## FasterLIO

This is the code repository of FasterLIO by Chunge Bai, Tao Xiang, Yajie Chen, Haoqian Wang, Fang Zhang and Xiang Gao.

Faster-LIO is a light-weight Lidar-inertial odometry for lidar pose tracking and point cloud mapping. It is developed on
the base of [FastLIO2](https://github.com/hku-mars/FAST_LIO) and provides about 1.5-2x speed increase. It achieves
almost 1k-2k hz for solid-state lidars and over 100 hz for a typical 32 lines spinning lidar. Please refer to [our paper](./doc/faster-lio.pdf)
for more details.

Below is the FPS tested with avia and velodyne 32 spinning lidar from FastLIO2 and [NCLT](http://robots.engin.umich.edu/nclt/), where "AMD" is tested with  AMD R7 5800X and "Intel" tested with Intel Xeon Gold 5218.

<div  align="center">  
<img src="https://github.com/gaoxiang12/faster-lio/blob/main/doc/fps-fasterlio.png" width = "1080" align=center />
</div>

<div  align="center">  
<img src="https://github.com/gaoxiang12/faster-lio/blob/main/doc/faster-lio-nclt.png" width = "1080" align=center />
</div>

# Quick Start
## Docker 
```bash
git clone 
cd faster-lio/docker
docker-compose build
docker-compose up
```  
Before entering `docker-compose up` command, you should enter `xhost +local:docker` in the local terminal to enable docker to communicate with Xserver on the host.  

update `bag` file path in `.env`, then run.
```bash
catkin_make
source devel/setup.bash
roslaunch faster_lio mapping_avia.launch
```
Open another terminal, then play `bag` file.

### issue
`QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'` 

**Don't worry, it can run normally**. you can run `source /etc/profile&& source /opt/ros/noetic/setup.bash && source devel/setup.bash`, then `run roslaunch xxx`


## Dependency

FasterLIO is tested in Ubuntu 18.04 and Ubuntu 20.04. Please install the following libraries before compilation.

1. ROS (melodic or noetic)
2. glog: ```sudo apt-get install libgoogle-glog-dev```
3. eigen: ```sudo apt-get install libeigen3-dev```
4. pcl: ```sudo apt-get install libpcl-dev```
5. yaml-cpp: ```sudo apt-get install libyaml-cpp-dev```

## Compile

FasterLIO can be compiled by plain cmake or catkin_make. In Ubuntu 20.04, the compile step is relatively simple.

1. Plain cmake 

Use the following commands to build FasterLIO:

```bin/bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

Note: iVox type should be specified by cmake at compile time. By default we will use linear iVox.
Use ```cmake .. -DWITH_IVOX_NODE_TYPE_PHC=ON``` to build the FasterLIO with PHC iVox.

2. catkin_make

Clone this repository to your catkin workspace, e.g., ~/catkin_ws/src, then use catkin_make instead of the above cmake
commands. You can also specify the iVox type in catkin_make parameters.

After the compilation, you will get the a libfaster_lio.so and two executable files. If you choose plain cmake build,
they will be located in ./build/devel/lib/faster_lio. If you use catkin_make, you could run them with rosrun and
roslaunch.

3. Compile in Ubuntu 18.04 or earlier

Since FasterLIO use cpp 17 as its standard (which requires a higher version of g++), you should upgrade the compiler and
use the earlier tbb library provided in the thirdparty of this repo. Please follow the steps here:

- Upgrade the g++ compiler to 9.0 or higher by:

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install gcc-9
cd /usr/bin
sudo rm gcc g++
sudo ln -s gcc-9 gcc
sudo ln -s g++-9 g++
```

- unzip the tbb library in the thirdparty:

```bash
cd thirdparty
tar -xvf tbb2018_20170726oss_lin.tgz
```

- specify the tbb directory in the cmake stage:

```bash
cd ../build
cmake .. -DCUSTOM_TBB_DIR=`pwd`/../thirdparty/tbb2018_20170726oss
```

Then just build FasterLIO with make.

## Prepare the datasets

Download the avia/nclt bags in your computer:

- [avia bags](https://drive.google.com/drive/folders/1YL5MQVYgAM8oAWUm7e3OGXZBPKkanmY1?usp=sharing)
- [nclt bags](https://drive.google.com/drive/folders/1VBK5idI1oyW0GC_I_Hxh63aqam3nocNK)

The link of the dataset is outdated.  Please use the NCLT bags in SAD datasets: 
BaiduYun: https://pan.baidu.com/s/1ELOcF1UTKdfiKBAaXnE8sQ?pwd=feky access code: feky
OneDrive：https://1drv.ms/u/s!AgNFVSzSYXMahcEZejoUwCaHRcactQ?e=YsOYy2

## Run FasterLIO

FasterLIO can be called like a plain binary program. It can run in offline mode or online mode like any ros program.

- Offline mode

Call run_mapping_offline with parameters to specify the bag file and the config file like:

```bash
./build/devel/lib/faster_lio/run_mapping_offline --bag_file your_avia_bag_file --config_file ./config/avia.yaml
```

for avia. For NCLT, just run:

```bash
./build/devel/lib/faster_lio/run_mapping_offline --bag_file your_nclt_bag_file --config_file ./config/velodyne.yaml
```

Please replace "your avia bag file" with the path in your computer. FasterLIO will print the FPS and time usage when it
quits:

```shell
I0216 17:16:05.286536 26492 run_mapping_offline.cc:89] Faster LIO average FPS: 1884.6
I0216 17:16:05.286549 26492 run_mapping_offline.cc:91] save trajectory to: ./src/fast_lio2/Log/faster_lio/20120615.tum
I0216 17:16:05.286706 26492 utils.h:52] >>> ===== Printing run time =====
I0216 17:16:05.286711 26492 utils.h:54] > [     IVox Add Points ] average time usage: 0.0147311 ms , called times: 6373
I0216 17:16:05.286721 26492 utils.h:54] > [     Incremental Mapping ] average time usage: 0.0271787 ms , called times: 6373
I0216 17:16:05.286731 26492 utils.h:54] > [     ObsModel (IEKF Build Jacobian) ] average time usage: 0.00745852 ms , called times: 25040
I0216 17:16:05.286752 26492 utils.h:54] > [     ObsModel (Lidar Match) ] average time usage: 0.0298004 ms , called times: 25040
I0216 17:16:05.286775 26492 utils.h:54] > [ Downsample PointCloud ] average time usage: 0.0224052 ms , called times: 6373
I0216 17:16:05.286784 26492 utils.h:54] > [ IEKF Solve and Update ] average time usage: 0.342008 ms , called times: 6373
I0216 17:16:05.286792 26492 utils.h:54] > [ Laser Mapping Single Run ] average time usage: 0.530618 ms , called times: 6387
I0216 17:16:05.286800 26492 utils.h:54] > [ Preprocess (Livox) ] average time usage: 0.0267813 ms , called times: 6387
I0216 17:16:05.286808 26492 utils.h:54] > [ Undistort Pcl ] average time usage: 0.0810455 ms , called times: 6375
I0216 17:16:05.286816 26492 utils.h:59] >>> ===== Printing run time end =====
```

Point clouds will be saved to PCD/scans.pcd by default. 

- Online mode 
 
Online mode could be launched through rosrun/roslaunch/directly call. We use roslaunch as an example:

1. Launch faster-lio: ```roslaunch faster_lio mapping_avia.launch``` This will give you a rviz window.
2. Play the bags using ```rosbag play your bag file``` to see the online outputs.

# Acknowledgements

- We thank the authors of [FastLIO2](https://github.com/hku-mars/FAST_LIO), LOAM for their great jobs.
- Please cite our work if you are using FasterLIO in academic work. Bibtex is provided here (early-access version):

```
@ARTICLE{9718203,  
author={Bai, Chunge and Xiao, Tao and Chen, Yajie and Wang, Haoqian and Zhang, Fang and Gao, Xiang},  
journal={IEEE Robotics and Automation Letters},   
title={Faster-LIO: Lightweight Tightly Coupled Lidar-Inertial Odometry Using Parallel Sparse Incremental Voxels},   
year={2022},  
volume={7},  
number={2},  
pages={4861-4868},  
doi={10.1109/LRA.2022.3152830}}
```


- This work is supported by Idriver+ Technologies Co. Ltd.

# Known Issues

- iVox is somehow sensitive to the voxel size. Please enlarge the voxel size if FasterLIO is not stable in your
  application.
- set -march=native in cmake may cause a core dump in the PCL point cloud deconstruction step (Ubuntu 20.04, gcc 11, AMD
  5800X)(Ubuntu 18.04, gcc 9, i7-10750H), see
  https://stackoverflow.com/questions/61278204/segmentation-fault-when-deallocating-pclpointcloudpclpointxyzptr
