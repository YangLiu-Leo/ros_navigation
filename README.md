

move_base路径规划的实现流程：

http://wiki.ros.org/cn/navigation/Tutorials

http://wiki.ros.org/move_base?distro=kinetic

https://blog.csdn.net/weixin_42048023/article/details/83992879

https://www.cnblogs.com/shhu1993/p/6323699.html

## (一)　局部路径规划

### 一　DWA在速度空间进行速度采样

DWA算法代码实现流程：https://blog.csdn.net/peakzuo/article/details/86487923

DWA速度采样原理及matlab仿真代码：https://blog.csdn.net/heyijia0327/article/details/44983551

#### 1.1 速度采样空间

速度空间在代码里的表现形式是一个三维向量，三个维度分别代表vx vy vth，然后这个向量空间的每一个点都代表一组速度样本(vx vy vth)，每组速度样本都可以生成一条局部轨迹。

#### 1.2 DWA代码分析

**1. navigation/dwa_local_planner/src/dwa_planner.pp**　DWAPlanner::findBestPath() 

```
generator_.initialise()  利用速度的最大最小限制和采样间隔生成速度空间

scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);　遍历速度速度空间每一个点结合当前位置速度和目标位置，生成轨迹打分返回最优轨迹
```

**2. 供上面的函数调用的两个类**

**1) navigation/base_local_planner/src/simple_scored_sampling_planner.cpp**

```
SimpleScoredSamplingPlanner::findBestTrajectory() 遍历速度速度空间每一个点结合当前位置速度和目标位置，生成轨迹打分返回最优轨迹，被1)里面的函数调用

SimpleScoredSamplingPlanner::scoreTrajectory()　使用多个打分类依次对一条轨迹进行打分
```

**2) navigation/base_local_planner/src/simple_trajectory_generator.cpp**　

```
SimpleTrajectoryGenerator::initialise() 生成速度空间

SimpleTrajectoryGenerator::nextTrajectory()  根据一组速度生成一条轨迹
```



### 二　DWA对每一对速度样本生成的一条轨迹进行打分

move_base/DWA源码分析（实现原理和打分函数分析）： https://www.cnblogs.com/sakabatou/p/8297479.html

**打分原则：**对一条轨迹打分，代价越低，其物理意义是使得局部轨迹更贴近全局轨迹、避开障碍物、终点朝向与目标点朝向一致

这里说的全局轨迹，是先给机器人一个世界坐标系的目标点，经过tf转换将目标点坐标转换为机器人坐标系坐标，再用在全局路径规划器(A*)生成的全局轨迹

在computeVelocityCommands()计算速度前，会先将全局路径映射到局部地图坐标系下（通常为“odom”），将较长的全局路径映射并截断到局部地图内（即坐标系转换为局部地图，且范围完全在局部地图内，超出地图的则抛弃）。

两个坐标系的点进行TF坐标转换：https://blog.csdn.net/weixin_38145317/article/details/83956463

tf进行位置查询：https://blog.csdn.net/crazyquhezheng/article/details/49124115

#### 2.1 局部轨迹与全局轨迹距离打分

##### 1. 网格计算地图　navigation/base_local_planner/inlude/base_local_planner/

根据局部轨迹与全局轨迹和局部目标点的距离打分信息（path distance and goal distance）

**1) map_cell.h定义了网格的数据结构**

网格地图的目的是维护一个跟局部地图(cost_map)尺寸对应的路径计算地图（MapGrid类型），地图中的每个元素为MapCell类型，该类型记录了该元素距离局部目标点的距离信息或者距离全局路径的距离信息（网格像素距离）。

```
  class MapCell{
    public:
      unsigned int cx, cy; ///< @brief Cell index in the grid map

      double target_dist; ///< @brief Distance to planner's path

      bool target_mark; ///< @brief Marks for computing path/goal distances

      bool within_robot; ///< @brief Mark for cells within the robot footprint
  };
  
```

**2) map_grid和costmap的坐标和网格索引坐标的转换**

在base_local_planner/src/map_grid_cost_function.cpp中 MapGridCostFunction::**scoreTrajectory**(Trajectory &traj)

```
  costmap_2d::Costmap2D* costmap_;
  base_local_planner::MapGrid map_;
  
  costmap_->worldToMap(px, py, cell_x, cell_y);　全局坐标转化成网格索引坐标
  grid_dist = getCellCosts(cell_x, cell_y);　调用了　grid_dist = map_(cell_x, 				cell_y).target_dist;
```

**3) map_grid.h及cpp里定义了网格地图和距离计算方法**

std::vector<MapCell> map_;　

还有一系列操作网格地图的方法。

局部轨迹与全局轨迹和局部目标点的距离的计算通过两个函数实现，根据is_local_goal_function参数可选：

先把全局路径上的点依次放入机器人当前位置旁边的cost_map里，

MapGrid::setTargetCells()　方式一：记录全部落在cost_map内的全局路径点，是用来计算轨迹路径计算地图中所有点到**局部地图路径**的最短距离

apGrid::setLocalGoal()　方式二：只记录和costmap边界相交的全局路径点作为goal，计算轨迹路径计算地图中所有点到**局部地图路径末端点(goal)**的最短距离

MapGrid::computeTargetDistance() 最后计算并更新上面记录的目标点及目标周围点的网格距离信息



##### 2.路径距离打分　navigation/base_local_planner/inlude/base_local_planner/

**1) 预处理**　在map_grid_cost_function.cpp

MapGridCostFunction::prepare()　根据不同的打分参数配置，决定记录距离目标点的距离信息或者距离路径的距离信息（网格像素距离）

**2) 打分函数　(类实例化为以下四个对象)**

MapGridCostFunction::scoreTrajectory(Trajectory &traj)

**path_costs_（打分项的stop_on_failure参数为true）**

依次获取路径中的各个点坐标，并尝试将坐标（世界坐标）转为地图坐标（计算网格图的索引），从路径计算地图中找到距离局部地图路径的距离，并将该距离（像素级）值作为该点的得分。

**alignment_costs_**

该打分项和path_costs_基本一样啊，只是参考的是机器人前向点和局部地图路径的距离，而不是机器人原点和局部地图路径的距离。

**goal_costs_（打分项的stop_on_failure参数为true）**

该打分项的思路与path_costs_的打分方式类似，只是计算的是点到局部目标点(goal)的距离信息。

**goal_front_costs_**

该打分项和goal_costs_基本一样，只是参考的是机器人前向点和局部目标点的距离，而不是机器人原点和局部目标点的距离。



#### 2.2 局部轨迹和障碍物距离打分

在base_local_planner/src/costmap_model.cpp中实现的footprintCost函数

在obstacle_cost_function.cpp　ObstacleCostFunction::footprintCost()里调用footprintCost()函数

**1.实现流程**

对轨迹上的每一点计算出对应的footprint（利用了x, y ,theta, footprint四个信息计算）进行footprintCost计算，然后在该点通过cost_map直接读取cost，返回二者里较大的作为最终的cost。

在进行对轨迹各个点计算footprintCost之前，会先计算缩放因子。如果当前平移速度小于scaling_speed，则缩放因子为1.0，否则，缩放因子为(vmag - scaling_speed) / (max_trans_vel - scaling_speed) * max_scaling_factor + 1.0，目的是保证速度大于某个阈值的时候放大footprint，根据速度改变打分效果，但是目前好像未使用。

**2.不同碰撞检测方式比较**

这里使用代价地图的好处，就是可以直接通过世界坐标转换为珊格地图索引坐标，然后查询该点及机器人在该点周围的footprint是否碰撞

如果维护costmap比较麻烦，也可以使用我们现有的碰撞检测方式，即每个轨迹点和所有的点云坐标计算欧式距离。

costmap的更新机制：　https://www.cnblogs.com/sakabatou/p/8297736.html

Bresenham算法画直线原理：https://blog.csdn.net/yzh1994414/article/details/82860187

#### 2.3 轨迹摆动打分

在dwa_local_planner/src/dwa_planner.cpp中的 DWAPlanner::findBestPath()中

1.摆动打分

调用scored_sampling_planner_.findBestTrajectory()根据机器人当前定位点生成一条最优轨迹，在这里打分的时候是拿上一次计算得到的摆动标志位进行摆动打分

2.摆动标志位更新

在得到最优轨迹以后，调用oscillation_costs_.updateOscillationFlags()，更新摆动标志位，给下个周期轨迹打分用。

如果在某些方向上有了限制，则检查是否能够复位标志位，只有当机器人移动一定位置后才运行reset标志位；

如果某方向（直行　侧向　旋转）的当前方向和规划速度方向不一致，则置位该方向的速度摆动标志位。

