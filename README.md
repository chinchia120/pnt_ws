# pnt_ws

## Overview
For 114-1 Positioning Navigation and Timing Technologies Theory and Application Lab Course

## Information
**Author**: Chin-Chia, Hsu, POINT Lab, Geomatics NCKU, Taiwan  
**Email**: chinchia120@geomatics.ncku.edu.tw  
**Update**: 2025/12/22


## Recommended System
**Ubuntu** : 22.04  
**ROS2** : Humble


## PNT Lab Course Link
[PNT Lab1 - Fundmantal and ROS](https://hackmd.io/@chinchia120/PNT-Lab1)  
[PNT Lab3 - GNSS in ROS](https://hackmd.io/@chinchia120/PNT-Lab3)  
[PNT Lab5 - GNSS/IMU in ROS](https://hackmd.io/@chinchia120/PNT-Lab5)  
[PNT Lab6 - Visual-Inertial Odometry in ROS](https://hackmd.io/@chinchia120/PNT-Lab6)  
[PNT Lab8 - LiDAR and Vehicle in ROS](https://hackmd.io/@chinchia120/PNT-Lab8)  


## Project Structure
```
src/
├── common/
│   ├── coordinate_transform/       # Coordinate Transform from WGS84 to Local coordinate
│   ├── novatel_converter/          # Convert NovAtel custom message to common message
│   └── path_display/               # Visualize path from localization algorithm
├── config
│   ├── iop_config_basler.yaml      # Config for Camera IOP
│   └── vins_config_basler_330.yaml # Config for VINS-Fusion-ROS2
│   ├── libtps_config_330.txt       # Config for NavFusion
├── launcher
│   └── pnt_launch/                 # Launch Node and RViz2
├── localization
│   └── VINS-Fusion-ROS2/           # Submodule of VINS-Fusion-ROS2
├── sensing
│   ├── basler_poser/               # Visualize image from Basler
│   └── velodyne_poser/             # Visualize pointcloud2 from Velodyne
├── vehicle
│   └── autoware_vehicle/           # Submodule of autoware_vehicle
├── .gitignore
├── .gitmodules
└── README.md
```