# Multi-Agent Path Planning (MAPF) in ROS

## Introduction
In order to verify the multi-agent path planning algorithms on **ROS**, 
this repository writes a **ROS wrapper** on the core code of some mapf algorithms(which mainly come from [HERE](https://github.com/whoenig/libMultiRobotPlanning)) as **ros plugins**. At the same time, the content of [this repository](https://github.com/atb033/multi_agent_path_planning) provides a lot of help.

The following algorithms are currently implemented:

+ Conflict-Based Search (CBS)
+ Enhanced Conflict-Based Search (ECBS)
+ Prioritized Planning using SIPP(example code for SIPP, the code to check swap has not been written yet)

## Build

The same process as the ros code package, just:

```catkin_make```

## Example

### Conflict-Based Search (CBS)

![](./doc/cbs.gif)


## Introduction of Code Structure
### ROS plugin picture

![](./doc/plugins_pic.png)

