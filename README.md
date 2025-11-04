# Multi-Layer-Path-Planning-for-Complete-Structural-Inspection-using-UAV
An inspection path planning algorithm that utilizes mixed viewpoint generation approach, proposed in the paper "[Multi-Layer Path Planning for Complete Structural Inspection using UAV](https://doi.org/10.3390/drones9080541)".

![alt text](https://github.com/Canning-Tong/Multi-Layer-Path-Planning-for-Complete-Structural-Inspection-using-UAV/blob/main/Flow_chart.png "Flow Chart")

# Input file
As a model-based planning approach, the algorithm requires the model of the inspection target in order to plan the appropriate inspection path. Here list the required files:
1. Model of the inspection target in .stl format. Isotropic explicit remeshing should be applied to the model to regulate the aspect ratio for the triangular patches. Can be prepared using [Meshlab](https://www.meshlab.net/#description)
2. Convex hull of the inspection target, in .stl format. Can be prepared using [Meshlab](https://www.meshlab.net/#description)

# Parameter Tuning
Several parameters require tuning to suit specific inspection scenario. They are listed below and explanined accordingly. 

1. D: The safety/ viewing distance, in meter. 
2. FOV: FOV of the onboard camera, in deg. 
3. vert_overlap*: Overlapping rate, determine the viewpoint density
4. height_lower*: Lower height limit for visibility calculation. Mesh under this level will not be counted toward the overall visibility.
5. height_upper*: Higher height limit for visibility calculation. Mesh above this level will not be counted toward the overall visibility. 
6. cov_perct*: Coverage percentage requirement to terminate the gap-filling viewpoint generation process. 
7. no_layer*: No. of layer to divide the final viewpoints into.

*: Adjust based on the inspection target

# Usage
1. Add all files, including stlTools and mesh files into MATLAB's path
2. Run Mixed_viewpoint_generation.m

# Reference
````
@Article{drones9080541,
AUTHOR = {Tong, Ho Wang and Li, Boyang and Huang, Hailong and Wen, Chih-Yung},
TITLE = {Multi-Layer Path Planning for Complete Structural Inspection Using UAV},
JOURNAL = {Drones},
VOLUME = {9},
YEAR = {2025},
NUMBER = {8},
ARTICLE-NUMBER = {541},
URL = {https://www.mdpi.com/2504-446X/9/8/541},
ISSN = {2504-446X},
ABSTRACT = {This article addresses the path planning problem for complete structural inspection using an unmanned aerial vehicle (UAV). The proposed method emphasizes the scalability of the viewpoints and aims to provide practical solutions to different inspection distance requirements, eliminating the need for extra view-planning procedures. First, the mixed-viewpoint generation is proposed. Then, the Multi-Layered Angle-Distance Traveling Salesman Problem (ML-ADTSP) is solved, which aims to reduce overall energy consumption and inspection path complexity. A two-step Genetic Algorithm (GA) is used to solve the combinatorial optimization problem. The performance of different crossover functions is also discussed. By solving the ML-ADTSP, the simulation results demonstrate that the mean accelerations of the UAV throughout the inspection path are flattened significantly, improving the overall path smoothness and reducing traversal difficulty. With minor low-level optimization, the proposed framework can be applied to inspect different structures.},
DOI = {10.3390/drones9080541}
}
````
