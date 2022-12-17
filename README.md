# DMP-Winter-Hackathon

[Dynamic Map Platform (DMP)](https://www.dynamic-maps.co.jp/en/) is a competition event to have Engineers and Designers to develop a service in a short period using the Dynamic Map Platform’s High-Precision 3D Map data.

## ASAP Team Proposal
視覚障害者支援のためのスマートスティックアプリケーション ～高精度3D地図によってバリアフリー社会をつくろう～
  
High-Precision 3D Map Can Make A Humanistic Society: A Smart Cane Application for Blind Person Assistance

## File Structure
- ros/dmp_package: Contain the developed ROS package for smart cane application.
- vector_maps: Contain the vector maps in [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) Format edited based on DMP's HD map and Tokyo Tech's HD map.
- waypoints: Contain the waypoints recoreded after a successful localization on HD map by [NDT matching](https://github.com/AbangLZU/Autoware/blob/master/ros/src/computing/perception/localization/packages/lidar_localizer/nodes/ndt_matching_monitor/README.md).
- rviz_config: Contain the visulization configuration file of Rviz for different scenarios.

## Demo I: Vehicle Lane Invasion Awareness
![movie](https://github.com/ZongdianLi/DMP-Winter-Hackathon/blob/main/demo%201.gif)

Main code: ros/dmp_package/scripts/distance2lane.py

After moving to your workspace, compling and sourcing the environtal variables, run:
```
rosrun dmp_package distance2lane.py
```

Then, launch autoware.ai Runtime Manager from where you can import HD map, set TFs and enable NDT matching:
```
cd autoware.ai
source install/setup.bash
roslaunch runtime_manager runtime_manager
```

Finally, visualize by importing the rviz setting under rviz_config/lane_awareness.rviz
```
rviz -d ./rviz_config/lane_awareness.rviz
```

## Demo II. Blind Track Deviation Awareness
![movie](https://github.com/ZongdianLi/DMP-Winter-Hackathon/blob/main/demo%202.gif)

Main code: ros/dmp_package/scripts/distance2track.py

Enter the workspace, source the environmental variables and run:
```
rosrun dmp_package distance2track.py
```

Launch autoware.ai; except changing HD map do same settings as the Demo I.

Visualize by importing the rviz setting under rviz_config/blindtrack_awareness.rviz
```
rviz -d ./rviz_config/blindtrack_awareness.rviz
```


## Contact
Zongdian Li - [lizd@mobile.ee.titech.ac.jp](lizd@mobile.ee.titech.ac.jp) 
