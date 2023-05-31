# Cartesian Planner ROS Package

C++/ROS Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
Frenet Frame: A Cartesian-based Trajectory Planning Method" published in IEEE Trans.
Intelligent Transportation Systems by Bai Li, Yakun Ouyang, Li Li, and Youmin Zhang.

![OnRoadPlanning](resources/static.png)

## Installation

Requirements

* ROS Melodic or later
* Python3


Install packages required by Ipopt

```shell
sudo apt-get install gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev
```

Clone repository to any catkin workspace and compile workspace

```shell
cd ~/catkin_ws/src
git clone https://github.com/libai1943/CartesianPlanner.git cartesian_planner
cd .. && catkin_make
source devel/setup.bash
```

**OPTIONAL**: build and install Harwell Subroutine Library (HSL) (recommended for better performance)

```shell
git clone https://github.com/coin-or-tools/ThirdParty-HSL.git

# Obtain a tarball with HSL source code from http://www.hsl.rl.ac.uk/ipopt/ and unpack this tarball
tar -zxvf coinhsl-x.y.z.tar

# Rename the directory `coinhsl-x.y.z` to `coinhsl`, or set a symbolic link:
ln -s coinhsl-x.y.z coinhsl

./configure
make
sudo make install

# create symlink for Ipopt
sudo ln -s /usr/local/lib/libcoinhsl.so /usr/local/lib/libhsl.so
# Re-build workspace
cd ~/catkin_ws && catkin_make -DWITH_HSL=ON
```

## Example



https://user-images.githubusercontent.com/85840949/150943617-f949d10d-c1be-424f-9530-1a21a5c67eef.mp4



Example test case with 6 pedestrians, 3 moving vehicles and 2 static vehicles.

```shell
roslaunch cartesian_planner pedestrian_test.launch
```

**Click anywhere in Rviz window with the `2D Nav Goal` Tool to start planning.**

---

Generate and run new random case:

```shell
roslaunch cartesian_planner random_pedestrian_test.launch
```


## Acknowledgement

[CASADi](https://github.com/casadi/casadi)

Special thanks to [Baidu Apollo](https://github.com/ApolloAuto/apollo) for common math libraries

---

Copyright (C) 2022 Bai Li and Yakun Ouyang

Users must cite the following article if they use the source codes to conduct simulations in their new publications.
Bai Li, Yakun Ouyang, Li Li, and Youmin Zhang, “Autonomous driving on curvy roads without reliance on Frenet frame: A Cartesian-based trajectory planning method,” IEEE Transactions on Intelligent Transportation Systems, vol. 23, no. 9, pp. 15729 - 15741, 2022. available at https://doi.org/10.1109/TITS.2022.3145389
