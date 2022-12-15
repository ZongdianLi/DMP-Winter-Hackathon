# DMP-Winter-Hackathon

[Dynamic Map Platform (DMP)](https://www.dynamic-maps.co.jp/en/) is a competition event to have Engineers and Designers to develop a service in a short period using the Dynamic Map Platform’s High-Precision 3D Map data.

## ASAP Team Proposal
視覚障害者支援のためのスマートスティックアプリケーション
  ～高精度3D地図によってバリアフリー社会をつくろう～
  
High-Precision 3D Map Can Make A Humanistic Society: A Smart Cane Application for Blind Person Assistance

## File Structure
- ros/dmp_package: Contain the developed ROS package for smart cane application.
- vector_maps: Contain the vector maps in [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) Format edited based on DMP's HD map and Tokyo Tech's HD map.
- waypoints: Contain the waypoints recoreded after a successful localization on HD map by [NDT matching](https://github.com/AbangLZU/Autoware/blob/master/ros/src/computing/perception/localization/packages/lidar_localizer/nodes/ndt_matching_monitor/README.md).

## Demo I: Vehicle Lane Invasion Awareness
Main code: ros/dmp_package/scripts/distance2lane.py

## Demo II. Blind Track Deviation Awareness

## Contact
Zongdian Li - [lizd@mobile.ee.titech.ac.jp](lizd@mobile.ee.titech.ac.jp) 
