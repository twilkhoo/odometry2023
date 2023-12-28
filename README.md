# Odometry
### Overview
Odometry is the usage of sensors to estimate an object's (in this case- a robot's) position. 

### Usage
The entirety of the odometry implementation is in `odom.[h/cpp]`, under the `src` dir. 

### Results
Many VEX teams who use PROS also use the OkapiLib API- it provides complete odometry and motion profiling algorithms out of the box, and teams only need to define some robot dimensions. I wanted to test if my odometry implementation would perform better than the OkapiLib's odometry class, so the test was to move the robot in an 8ft x 8ft square, come back to the origin, and face the initial orientation. With a perfect odometry implementation, the robot should read (x,y,deg) of (0,0,0) at this point, but this unrealistic. My odometry class and the OkapiLib Odom class were run concurrently, set up with the exact same robot dimensions. The results of both my algorithm and OkapiLib were the following:
![Odometry Tests](/IMG_6067.jpg)


The ending point for my implementation was 1.791756" away from the origin, while OkapiLib's was 2.286435" away, implying a ~21.6% increase in (x,y) accuracy. The biggest improvement, however, was the angle drift from 6.357344deg using OkapiLib to 1.829856deg using my algorithm, and this improvement in heading accuracy must have played a role in the improvement of (x,y) accuracy too. This is likely because my heading algorithm is slightly different than the one implemented in OkapiLib (non-iterative). I conducted this test more times with different movement patterns and the results were similar.

### Next Steps
More motion algos! A robust odometry class serves no purpose without equally robust motion algorithms. I've implemented basic ones just to demonstrate how the odometry class can be used (PID TurnToPoint, PID GoToPoint), but other popular algorithms that many teams try to implement include:
* Pure Pursuit Controllers
* 2D Motion Profiling
