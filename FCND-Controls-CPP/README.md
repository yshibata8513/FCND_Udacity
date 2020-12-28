
## Building a Controller Project 
In the following, I will explain how my controller meets each rubic point.

### Implemented body rate control in C++.
I simply implemented propotional controller  <code> BodyRateControl</code> which uses error with respect to body rates. 
Moment of inertia was considered when I transformed desired rates of changes of body rates to desired moments.


```
  const V3F pqrErr    = pqrCmd - pqr;  
  const V3F pqrDotCmd = kpPQR*pqrErr;  
  const V3F MoI(Ixx,Iyy,Izz);  
  momentCmd = MoI*pqrDotCmd;
```
### Implement roll pitch control in C++.
I implemented roll pitch controller in <code>RollPitchControl</code>.First I used drone's mass to transform collective thrust to corresponding acceleration.

```
const float c = -collThrustCmd/mass;
```

Second I calculated desired values of <code> R13,R23</code> ,which are elements of Rotation matrix,  and used propotional feedback to calculate rate of change of these values .

```
  const float bxCmd = CONSTRAIN(accelCmd.x/c, -maxTiltAngle, maxTiltAngle);
  const float byCmd = CONSTRAIN(accelCmd.y/c, -maxTiltAngle, maxTiltAngle);
  const float bxErr = bxCmd - R(0,2);
  const float byErr = byCmd - R(1,2);
  const float bxDotCmd = kpBank * bxErr;
  const float byDotCmd = kpBank * byErr;
```
Finally I transformed these rate of changes to body rate.Notation and implementation follow the reference suggested in a lecture. 
```
  const float pCmd = ( R(1,0)*bxDotCmd - R(0,0)*byDotCmd )/R(2,2);
  const float qCmd = ( R(1,1)*bxDotCmd - R(0,1)*byDotCmd )/R(2,2);
```

[Angela P. Schoellig, Clemens Wiltsche, Raffaello D'Andrea "Feed-forward parameter identification for precise periodic quadrocopter motions"](https://ieeexplore.ieee.org/document/6315248?section=abstract)


### Implement altitude controller in C++.
I implemented altitude contorller in <code>AltitudeControl</code>.First I calculated desired acceleration as the sum of the feedforward term and feedback(PID) terms , where  I calculated desired velocity as the sum of <code>velZCmd</code> and propotional feedback of position error.Additionaly the integral term is introduced to deal with the misspesification of weight.
```
  const float posZErr = posZCmd - posZ;
  integratedAltitudeError += dt*posZErr;
  float velZCmdFF = kpPosZ * posZErr + velZCmd;
  velZCmdFF = CONSTRAIN(velZCmdFF, -maxAscentRate, maxDescentRate);
  const float velZErr = velZCmdFF - velZ;
  const float accelZCmdTotalFB =  kpVelZ*velZErr + KiPosZ*integratedAltitudeError;
  const float accelZCmdTotal = accelZCmd + accelZCmdTotalFB - 9.81;
```
Second acceleration was corrected to account for the tilt of the drone's axis.
```
  const float bz = R(2,2);
  const float accelZBody = accelZCmdTotal/bz;
```
Finally I transformed acceleration to thrust using drone's mass.
```
thrust = -mass*accelZBody;
```

### Implement lateral position control in C++.
I implemented lateral position contorller in <code>LateralPositionControl</code>.
I calculated desired acceleration as the sum of the feedforward term and feedback(PD) terms.In the middle of the calculation, the absolute values of target velocity and target acceleration are limited by the maximum and minimum values.


### Implement yaw control in C++.
I implemented lateral position contorller in <code>YawControl</code>.I implemented a simple propotional controller.

### Implement calculating the motor commands given commanded thrust and moments in C++.

The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments.

I implemented the function calculating the motor commands given commanded thrust and moments in <code>GenerateMotorCommands</code>.
First I transformed desired moments to forces using the dimensions of the drone.
```
  const float c = collThrustCmd;
  const float Mp = momentCmd.x;
  const float Mq = momentCmd.y;
  const float Mr = momentCmd.z;

  const float c_bar = c;
  const float p_bar = Mp/l;
  const float q_bar = Mq/l;
  const float r_bar = Mr/kappa;
```
Second I calculated thrusts of each propeller which generate desired moments and collective thrust.
```
  cmd.desiredThrustsN[0]  = (c_bar + p_bar + q_bar - r_bar)/4.0;
  cmd.desiredThrustsN[1]  = (c_bar - p_bar + q_bar + r_bar)/4.0;
  cmd.desiredThrustsN[2]  = (c_bar + p_bar - q_bar + r_bar)/4.0;
  cmd.desiredThrustsN[3]  = (c_bar - p_bar - q_bar - r_bar)/4.0;
```

### Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory.

My controller was able to pass the test in all scenarios using the same set of parameters .Below are images showing that the drone has passed each test.

#### Senario2
![senario2](./fig/senario2.png?raw=true)
#### Senario3
![senario3](https://github.com/yshibata8513/FCND_Udacity/tree/master/FCND-Controls-CPP/fig/senario3.png?raw=true)
#### Senario4
![senario4](https://github.com/yshibata8513/FCND_Udacity/tree/master/FCND-Controls-CPP/fig/senario4.png?raw=true)
#### Senario5
![senario5](https://github.com/yshibata8513/FCND_Udacity/tree/master/FCND-Controls-CPP/fig/senario5.png?raw=true)