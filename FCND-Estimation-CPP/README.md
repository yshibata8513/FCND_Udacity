
## Building a Estimator Project 
In the following, I will explain how my estimator and controller meet each rubic point.

### Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.
I calculated standard the deviation of both GPS X data and Accelerometer X data.The results were 0.68 and 0.48 respectively and these values made the test passed.

### Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.

I replaced the linearized integral with a more precise nonlinear integral in the part where the current attitude is predicted by combining the gyroscopic measurements with the attitude of one time step ago.  

```
  auto qt = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
  auto qt_bar = qt.IntegrateBodyRate(V3D(gyro.x, gyro.y, gyro.z), dtIMU);

  const float predictedPitch = qt_bar.Pitch();
  const float predictedRoll = qt_bar.Roll();
  ekfState(6) = qt_bar.Yaw();
  if (ekfState(6) >  F_PI) ekfState(6) -= 2.0*F_PI;
  if (ekfState(6) < -F_PI) ekfState(6) += 2.0*F_PI;

```

### Implement all of the elements of the prediction step for the estimator.
I implemented prediction step in <code>Predict()</code>,in which prediction of next state and calculation of its uncertainty have beed done.
First I implemented <code>PredictState()</code> to calculate predicted mean of next state ,where current state , acceleration transformed to world the flame and gravitational acceleration were used to calculate next state.

```
  const V3F gravity(0.0,0.0,-9.81);
  const V3F accel_with_gravity = attitude.Rotate_BtoI(accel) + gravity;

  predictedState(0) = curState(0) + dt*curState(3);
  predictedState(1) = curState(1) + dt*curState(4);
  predictedState(2) = curState(2) + dt*curState(5);

  predictedState(3) = curState(3) + dt*accel_with_gravity.x;
  predictedState(4) = curState(4) + dt*accel_with_gravity.y;
  predictedState(5) = curState(5) + dt*accel_with_gravity.z;

  predictedState(6) = curState(6);
```
Second  I implemented a helper function <code>GetRbgPrime()</code> to calculate derivertive of Rotation matrix with respect to yaw to leverage it to calculate the Jacobian of next state with respect to current state <code>gPrime</code>. 

```
  const float phi   = roll;
  const float theta = pitch;
  const float psi   = yaw;

  RbgPrime(0,0) =  -cos(theta)*sin(phi);
  RbgPrime(0,1) =  -sin(phi)*sin(theta)*sin(psi) - cos(phi)*cos(psi);  
  RbgPrime(0,2) =  -cos(phi)*sin(theta)*sin(psi) + sin(phi)*cos(psi);
  RbgPrime(1,0) =  cos(theta)*cos(psi);
  RbgPrime(1,1) =  sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi);
  RbgPrime(1,2) =  cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);

  RbgPrime(2,2) = RbgPrime(2,1) =RbgPrime(2,0) = 0.0;
```

Lastly I implemented calculation updated covariance matrix of next step following Kalman Filters' update rule. 
```
  VectorXf accel_v(3);
  accel_v(0) = accel.x;
  accel_v(1) = accel.y;
  accel_v(2) = accel.z;

  const int d_row = 3;
  
  const VectorXf DvelDphi = RbgPrime*accel_v*dt;
  
  gPrime(0,3) = gPrime(1,4) = gPrime(2,5) = dt;

  for(int i=0;i<3;i++) {
    gPrime(i+d_row,6) = DvelDphi(i);
  }
  
  ekfCov = gPrime*ekfCov*gPrime.transpose() + Q;
```

### Implement the magnetometer update.
I implemented update step with respect to magnetometer observation in <code>UpdateFromMag()</code>.This implementation was straightforward because the observation value yaw was included in state variables , and prediction of observation value could be done by simply extracting yaw element from current state.

```
  zFromX(0)   = ekfState(6);

  const float diff = z(0) - ekfState(6);
  if (diff > F_PI) {
    z(0) -= 2.0*F_PI;
  }else if(diff < -F_PI){
    z(0) += 2.0*F_PI;
  }

  hPrime(0,6) = 1.0;
```


### Implement the GPS update.
I implemented update step with respect to magnetometer observation in <code>UpdateFromGPS()</code>.This implementation was also straightforward in the same reason of  <code>UpdateFromMag()</code>,and could be carried out by simply extracting six elements corresponding to observation values from current state.

```
  for(int i=0;i<6;i++) {
    hPrime(i,i) = 1.0;
    zFromX(i) = ekfState(i);
  }
```

### Meet the performance criteria of each step.
My estimator was able to successfully meet the performance criteria with the controller provided.


### De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.
My controller and estimator was able to pass the test in the last scenarios.Here I have not needed to detune parameters of controller though a small amount of fluctuation occured.
![own_controller](./images/senario11_my_controller.png?raw=true)
