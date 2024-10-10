![Project Pegasus](assets/qaudVis.png)

# Project Pegasus: A Search and Rescue/Firefighting Robotic System

**Project Pegasus** is a conceptual multi-pedal robotic system designed for search and rescue operations and firefighting in hazardous environments. This system focuses on reducing the risk to human emergency workers by autonomously navigating and extinguishing fires, identifying civilians, and mapping environments that would otherwise be too dangerous for people.

## Overview
Project Pegasus utilizes a quadruped or hexapod robot to traverse complex terrains such as collapsed buildings or fire-damaged environments. The robot is equipped with a manipulator that can interact with obstacles—such as opening doors or moving debris—and an attachable hose system for extinguishing fires. Designed to be weatherproof and heat-resistant, the robot can operate in extreme conditions, assisting emergency personnel in critical situations.

## Functional Requirements
1. **Autonomous Navigation**  
   The robot must autonomously navigate a 3D environment with obstacles and basic waypoints to reach specified objectives.
   
2. **Fire Detection**  
   Using onboard cameras and thermal sensors, the robot will identify fire locations within the environment and pinpoint their coordinates in 3D space.

3. **Fire Extinguishing Mechanism**  
   A gimbal-based hose system will allow the robot to direct water or CO2 at fires, effectively "extinguishing" them within the simulation.

4. **Real-Time Mapping**  
   The robot will map the environment in real-time, creating a memory map of the area as it navigates, recording vital information for situational awareness.

## Simulations
Due to the high-risk nature of real-world testing, simulations in ROS (Robot Operating System) will be used extensively to prototype and validate Project Pegasus. These simulations allow for controlled testing environments, ensuring features are implemented safely and cost-effectively.

## Justification for Multi-Pedal Robot
Opting for a quadruped or hexapod design provides a balance between speed, maneuverability, and traction. In environments with debris and uneven terrain, such as a collapsed or burning building, the robot's legs will allow it to navigate where wheeled robots would struggle.

## Key Sensors
| Sensor        | Module                           | Justification                       |
| ------------- | -------------------------------- | ----------------------------------- |
| **IMU**       | Accelerometer & Gyroscope        | Position & Navigation               |
| **RTC**       | Real-Time Clock                  | Time measurement                    |
| **Camera**    | Visual Computation               | Environmental Awareness             |
| **Thermal**   | Thermal Imaging                  | Fire Detection                      |
| **Temperature**| Temperature Sensors             | Safety and System Health Monitoring |
| **LIDAR**     | Laser Scanning                   | Object Detection & Navigation       |
| **Microphone**| Audio Input                      | People Detection                    |

## Objectives
- **Locate Fire**: The robot must locate and accurately identify fire sources in the environment.
- **Extinguish Fire**: Using its manipulator, the robot will direct its hose mechanism toward the fire and extinguish it.
- **Locate Civilians**: The robot will search rooms for civilians and mark their locations if detected.
- **Maneuver Through Rough Terrain**: By utilizing its onboard sensors, the robot will build an internal map and calculate optimal paths to avoid obstacles.
- **Auto Map Surroundings**: The robot will generate a detailed 3D map of the surroundings, including object detection and terrain analysis.

## Evaluation Criteria
| Criteria          | Full                          | Partial                          | Bare                          |
| ----------------- | ----------------------------- | -------------------------------- | ----------------------------- |
| **Mapping**        | Export point cloud, terrain map, and analysis | Interpret point cloud with some analysis | Interpret point cloud only |
| **Object Detection**| Detect fires, people, obstacles, debris, terrain, rooms | Detect fires, people, rooms, obstacles | Detect fires, people |
| **Behavior**      | Extinguish fires, locate people, and map surroundings | Extinguish fires, locate people while mapping | Map, locate people, and then extinguish fires |
