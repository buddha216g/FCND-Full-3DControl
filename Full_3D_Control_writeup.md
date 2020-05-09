# The Full 3D Control Project Writeup #



### Project Overview ###

The goal of this project is to design and build a cascade PID controller, which will control the quadrotors to fly the desired trajectory in the 3D environment.


### Implemented Controller ###
For the project, the majority of your code was  written in `src/QuadControl.cpp`.  This file contains all of the code for the controller that i developed.

All the configuration files for the controller and the vehicle are in the `config` directory.


### Introduction (scenario 1) ### 

When you run the simulator, you'll notice your quad is falling straight down.  This is due to the fact that the thrusts are simply being set to:

```
QuadControlParams.Mass * 9.81 / 4
```

Therefore, if the mass doesn't match the actual mass of the quad, it'll fall down.  

I tuned the `Mass` parameter in `QuadControlParams.txt` to 0.49 so as to make the vehicle more or less stay in the same spot.

See the Scenario1 output in the simulator snapshot below 

<p align="center">
<img src="https://github.com/buddha216g/FCND-Full-3DControl/blob/master/Simulator_Outputs/Scenario1.png"/>
</p>



### Body rate and roll/pitch control (scenario 2) ###

First, i implemented the body rate and roll / pitch control.  For the simulation, i used `Scenario 2`.  In this scenario, the quad wil be above the origin.  It is created with a small initial rotation speed about its roll axis.  My controller was able to stabilize the rotational motion and bring the vehicle back to level attitude.

To accomplish this, i:

1. Implemented body rate control

 - implemented the function `GenerateMotorCommands()`  : lines 74 to 93 in QuadControl.cpp (student code part)
 - implemented the function `BodyRateControl()` : 123 to 128 in QuadControl.cpp (student code part)
 - Tuned `kpPQR` in `QuadControlParams.txt` : value set to [90,90, 6]  (vehicle stopped spinning quickly but not overshoot)

After implementation the rotation of the vehicle about roll (omega.x) got controlled to 0 while other rates remained zero.  Note that the vehicle will keep flying off quite quickly, since the angle is not yet being controlled back to 0.  Also note that some overshoot will happen due to motor dynamics!.


2. Implement roll / pitch control

 - I implemented function `RollPitchControl()` : lines 157 to 174 in QuadControl.cpp (student code part)
 - I Tuned `kpBank` in `QuadControlParams.txt` : value set to 10 (minimize settling time but avoid too much overshoot)

The quad now levelled itself , though it’ll still be flying away slowly since i am not controlling velocity/position!  The vehicle angle (Roll) however, gets controlled to 0.


See the Scenario2 output in the simulator snapshot below 

<p align="center">
<img src="https://github.com/buddha216g/FCND-Full-3DControl/blob/master/Simulator_Outputs/Scenario2.png"/>
</p>

### Position/velocity and yaw angle control (scenario 3) ###

Next, I implemented the position, altitude and yaw control for your quad.  For the simulation, you used `Scenario 3`.  This  created 2 identical quads, one offset from its target point (but initialized with yaw = 0) and second offset from target point but yaw = 45 degrees.

 - implemented the function `LateralPositionControl()` : lines 262 to 285 in QuadControl.cpp (student code part)
 - implemented the function `AltitudeControl()`: lines 206 to 223 in QuadControl.cpp (student code part)
 - tuned parameters `kpPosZ` and `kpPosZ` :  kpPosZ = 25 and KiPosZ = 40
 - tuned parameters `kpVelXY` and `kpVelZ` : kpVelXY = 12.0 and kpVelZ = 9.0

The quads are now oing to their destination points and tracking error is going down (as shown below). However, one quad remains rotated in yaw.

Tips : Tune position control for settling time. Don’t try to tune yaw control too tightly, as yaw control requires a lot of control authority from a quadcopter and can really affect other degrees of freedom.  This is why you often see quadcopters with tilted motors, better yaw authority!

**Hint:**  For a second order system, such as the one for this quadcopter, the velocity gain (`kpVelXY` and `kpVelZ`) should be at least ~3-4 times greater than the respective position gain (`kpPosXY` and `kpPosZ`).

 - implemented the function `YawControl()` : lines 307 to 321 in QuadControl.cpp (student code part)
 - tuned parameters `kpYaw` and the 3rd (z) component of `kpPQR` : kpYaw = 2 and kpPQR = 90,90, 6



See the Scenario3 output in the simulator snapshot below 

<p align="center">
<img src="https://github.com/buddha216g/FCND-Full-3DControl/blob/master/Simulator_Outputs/Scenario3.png"/>
</p>



### Non-idealities and robustness (scenario 4) ###

In this part, I explored some of the non-idealities and robustness of a controller.  For this simulation, i used `Scenario 4`.  This is a configuration with 3 quads that are all are trying to move one meter forward.  However, this time, these quads are all a bit different:
 - The green quad has its center of mass shifted back
 - The orange vehicle is an ideal quad
 - The red vehicle is heavier than usual

1. Ran the controller & parameter set from Step 3.  Tweaked the controller parameters to work for all 3 (tip: relax the controller).

2. Edited `AltitudeControl()` to add basic integral control to help with the different-mass vehicle.

3. Tuned the integral control, and other control parameters until all the quads successfully move properly.  

See the Scenario4 output in the simulator snapshot below 

<p align="center">
<img src="https://github.com/buddha216g/FCND-Full-3DControl/blob/master/Simulator_Outputs/Scenario4.png"/>
</p>



### Tracking trajectories ###

Now that we have all the working parts of a controller, you put it all together and test it's performance once again on a trajectory.  For this simulation, i used `Scenario 5`.  This scenario has two quadcopters:
 - the orange one is following `traj/FigureEight.txt`
 - the other one is following `traj/FigureEightFF.txt` - for now this is the same trajectory.  
 
 See the Scenario5 output in the simulator snapshot below 

<p align="center">
<img src="https://github.com/buddha216g/FCND-Full-3DControl/blob/master/Simulator_Outputs/Scenario5.png"/>
</p>


## Evaluation ##

For All 5 scenarios, 

 - in the command line, at the end of each simulation loop, a **PASS** for each metric was displayed
 - on the plots, green box appeared on the plot notifying me of a **PASS**


### Performance Metrics ###

All 5 scenarios performed as per specifications outlined below:

 - scenario 2
   - roll should less than 0.025 radian of nominal for 0.75 seconds (3/4 of the duration of the loop)
   - roll rate should less than 2.5 radian/sec for 0.75 seconds

 - scenario 3
   - X position of both drones should be within 0.1 meters of the target for at least 1.25 seconds
   - Quad2 yaw should be within 0.1 of the target for at least 1 second


 - scenario 4
   - position error for all 3 quads should be less than 0.1 meters for at least 1.5 seconds

 - scenario 5
   - position error of the quad should be less than 0.25 meters for at least 3 seconds
