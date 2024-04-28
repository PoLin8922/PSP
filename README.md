## commom operation
- launch the realsense:
```
roslaunch realsense2_camera demo_pointcloud.launch
```

- launch controller ( whole pkg)
```
roslaunch controller controller.py
```

- rosrun individual pkg(exampel rosrun capture)
```
rosrun vision_capture2 vision_capture.py
rosservice call /vision_capture "scan: true"
```

- rosrun individual pkg(exampel rosrun path planning)
```
rosrun path_planning_ver1 path_planning_ver1
rosservice call /path_planning_ver1 "REQU_PP: true"
```

## system installation
- upgrade cmake -> https://blog.csdn.net/qq_36852840/article/details/128338724
- remember to reinstall ros after remove origin cmake
- libmobus
```
sudo apt-get install libmodbus-dev
sudo apt-get install modbus*
```
- Open3d
please follow the offical instruction(c++) -> https://www.open3d.org/docs/release/compilation.html
```
sudo pip install open3d
```