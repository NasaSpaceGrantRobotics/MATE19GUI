# MATE19 
[![Build Status](https://travis-ci.com/NasaSpaceGrantRobotics/MATE19.svg?branch=master)](https://travis-ci.com/NasaSpaceGrantRobotics/MATE19)  
Version: v0.1.0

## Table of Contents
* [Overview](#overview)
* [Requirements](#requirements)
* [Contributing](#contributing)
* [License](#license)

## Overview
The MATE19 repository contains all of ASU NASA Space Grant Robotics' software for the 2019 MATE competition season, including driverside GUI and input processing code and robotside motion planning and control code.

## Requirements
* Ubuntu 16.04.x
* ROS Kinetic
* Python 2.7

## Contributing
1. Clone this repository into ~/catkin_ws/src/
    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/NasaSpaceGrantRobotics/MATE19.git
    ```
2. Make the packages
    ```
    $ cd ~/catkin_ws
    $ catkin_make
    ```
3. Source ROS Environment variables (you likely want to add this to your .bashrc)
    ```
    $ source ~/catkin_ws/devel/setup.bash
    ```
4. Check that the packages are installed
    ```
    $ rospack find driverside
    $ rospack find robotside
    ```
5. Commit your changes to a feature branch, then make a pull request for merging your feature branch into master. Make sure your code follows team contribution guidelines.

## License
This repository is distributed under the terms of the [GNU GPL v3.0 License](https://choosealicense.com/licenses/gpl-3.0/).
