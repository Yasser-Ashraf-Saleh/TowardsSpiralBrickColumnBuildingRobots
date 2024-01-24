# TowardsSpiralBrickColumnBuildingRobots
In this project, we proposed a task-level approach for assembly of spiral brick columns. Our extensive computational simulations using the generalized models of spiral brick columns show the feasibility, the effectiveness and efﬁciency of our proposed approach.
# Overview 
Motivation
The motivation behind this project stems from the growing importance of automation in construction to overcome labour-intensive limitations and increase productivity. As the construction industry is poised to surpass $10 trillion by 2021, addressing the challenges through innovative and automated approaches becomes crucial. The project aims to fill a gap in the existing research by focusing on task-level planning for the assembly of spiral brick columns, a relatively unexplored area.

Problem Statment
The construction industry, a cornerstone of the global market, faces inherent challenges in labour productivity compared to other technology-intensive sectors. Automation in construction, particularly in the assembly of complex structures like spiral brick columns, holds the potential to enhance efficiency and productivity. The existing literature discusses the integration of robotics and additive manufacturing in construction, but there is a need to address specific challenges related to the assembly of spiral brick columns.

Objective
The objective of this project is to propose a task-level approach for the autonomous assembly of spiral brick columns. The focus is on leveraging computational simulations and generalized models to demonstrate the proposed approach's feasibility, effectiveness, and efficiency. The ultimate goal is to contribute to the advancement of automated construction methods, specifically targeting the assembly of aesthetically pleasing spiral brick columns.

# Proposed Approach

Overall Framework
The proposed approach aims to automate the assembly of user-defined spiral brick columns using a manipulator and incoming bricks from a belt conveyor. The approach consists of three main components:

Building Modeler: Computes the poses of bricks based on the dimensions and user-defined configurations of the spiral brick column.
Pose Estimator of Bricks: Computes the position and orientation of incoming bricks from the belt conveyor using readings from a depth camera located close to the conveyor.
Task Executioner: Determines the sequence of robot motion to enable effective pick and place of bricks.
In the subsequent subsections, we delve into the main concepts and interaction dynamics among these components.

2.2 Spiral Brick Models
The modeling of spiral brick columns is depicted in Fig. 2. Based on the geometry of the column's base and its configuration parameters, topologically distinct geometries are renderable. The formation of segments and their configurations play a crucial role, with parallel and orthogonal configurations having distinct mechanisms. Regular polygons and polynomial functions are also considered for rendering spiral columns, providing flexibility in design. These configurations enable the formation of aesthetically pleasing and structurally sound spiral brick columns.

2.3 Pose Estimator
To estimate the position and orientation of incoming bricks, a depth camera is fixed over the conveyor. Assuming calibrated camera intrinsics and known conveyor poses, point clouds of the perceived environment are computed using the pinhole camera model. Filtering and region of interest determination are performed, followed by fitting a plane using RANSAC and likelihood principles. Pose estimation of bricks is accomplished using the minimum bounding box method. This approach ensures accurate and efficient estimation of brick poses for subsequent manipulation.

2.4 Task Execution
The task planner plays a pivotal role in deciding the sequences of manipulator motion for effective pick and place of bricks. Given the poses of the brick and its target, the manipulator aligns and picks up the brick, plans a path for movement, executes the pick and place operation, and repeats the process until all bricks are placed in their desired positions within the spiral brick columns.

