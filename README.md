# Cartesian Planner ROS Package

 C++/ROS Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
 Frenet Frame: A Cartesian-based Trajectory Planning Method" published in IEEE Trans. 
 Intelligent Transportation Systems.



## Examples

```shell
roslaunch cartesian_planner pedestrian_test.launch
```

## Installation

Request **ma27** linear solver code from [HSL for IPOPT](https://www.hsl.rl.ac.uk/ipopt/), follow the installation instructions from [IPOPT
HSL autotools](https://github.com/coin-or-tools/ThirdParty-HSL).

Install deb package from [CASADi Releases](https://github.com/casadi/casadi/releases/tag/3.5.5).

```shell
sudo dpkg -i libcasadi-v3.5.5.deb
```

## Acknowledgement

[CASADi](https://github.com/casadi/casadi)

Special thanks to [Baidu Apollo](https://github.com/ApolloAuto/apollo) for common math libraries

---

 Copyright (C) 2022 Bai Li

 Users are suggested to cite the following article when they use the source codes.
 Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
 Frenet Frame: A Cartesian-based Trajectory Planning Method",
 IEEE Transactions on Intelligent Transportation Systems, 2022.


