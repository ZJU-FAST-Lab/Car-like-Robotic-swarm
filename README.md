# Decentralized Car-Like Robotic Swarm

## 0. Overview
Code in this repo is about the decentralized car-like robotic swarm in a simulation environment. Due to the authority, we replace the Carla map with a simple environment, where the MPC controller is also removed. 

**Related Paper**: 

[Decentralized Planning for Car-Like Robotic Swarm in Cluttered Environments](https://arxiv.org/abs/2210.05863), Changjia Ma, Zhichao Han, Tingrui Zhang, Jingping Wang, Long Xu, Chengyang Li, Chao Xu and Fei Gao, Accepted by **IROS 2023**.

The video link: [youtube](https://www.youtube.com/watch?v=qdYr3BKHsdM) and [bilibili](https://www.bilibili.com/video/BV11e4y1n7JL/?spm_id_from=333.999.0.0&vd_source=52c6d27efb21131ce8de5028bf3873c7).

<p align="center">
  <img src="figs/header.gif" width = "800"/>
</p>


## 1. Setup
All the Simulations are conducted in the Linux environment on a PC equipped with an Intel Core i7-10700 CPU and a GeForce RTX 3070 GPU.

Our software is developed and tested in Ubuntu 20.04 with ROS installed.

**Attention!** Map generator in this source code requires a higher version of python3, hence we do not recommend using Ubuntu 18 since the default version of python is python2.

## 2. Build on ROS

1. Create an empty new workspace and clone this repository to your workspace: 

```
git clone https://github.com/ZJU-FAST-Lab/Car-like-Robotic-swarm.git
cd Car-like-Robotic-swarm
```

2. Compile it.

```
catkin_make -DCMAKE_BUILD_TYPE=Release
```

**Attention!** When compiling, be sure to use **release** mode!(Just follow the above command)

## 3. Run

```
source devel/setup.bash
```

Then, run the command:

```
roslaunch traj_planner swarm.launch
```

Wait for about 2 seconds until the global map comes out.

Then, you can click the **2D Nav Goal** at **anywhere** in **RVIZ** to trigger the planning.

Here is an example:

<p align="center">
  <img src="figs/demo.gif" width = "800"/>
</p>

**Tip:** We recommend developers to use **rosmon** to replace the **roslaunch** since rosmon is more friendly to mulit-robot systems.

Please refer to [swarm formation](https://github.com/ZJU-FAST-Lab/Swarm-Formation) for more details of rosmon.

## 4. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

## 5. Acknowledgement

The trajectory representation is modified from [MINCO](https://github.com/ZJU-FAST-Lab/GCOPTER).

For a more detailed version of car-like robot planning, please see [Dftpav](https://github.com/ZJU-FAST-Lab/Dftpav).

## 6. Maintaince

For any technical issue, please contact Changjia Ma(changjiama@zju.edu.cn).

For commercial inquiries, please contact [Fei GAO](http://zju-fast.com/fei-gao/) (fgaoaa@zju.edu.cn).
