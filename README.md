# Gael Force Robotics Lib


## Credit
Gael Force V- 5327V

## Installation

> [!NOTE]
> Note that PROS is only support on ancient compiler

1. Download `gfrlib.zip` from the [latest release](https://github.com/AgilanSam/5327V/releases/latest)
2. Add the zip file to your project
3. Run `pros c fetch gfrlib.zip; pros c apply gfrlib@version` in the PROS integrated terminal

## Additional Versions

Want a different version of Eigen? Let us know by opening an issue [here](https://github.com/AgilanSam/5327V/issues/new)

## Perks
 > Multiple PID movements
 > Odometry & Odometry-Based movements

## PID struct 
> Use a PID controller in various robot applications such as lift, etc.

PID(kP, kI, kD, kImax, settleError, settleTime, maxSettleError, maxSettleTime, maxTime)
- Kp(proportional)
- kI(integral)
- kD(derivative)
- settleError(small settle error)
- settleTime(for small error)
- maxSettleError(for long distances)
- maxSettleTime(longest time it takes to settle)
- maxTime(how long the entire movement)
  
## Odometry
 > IME Odometry(adding Kalman Filters soon)
## Movement Functions
### chassis.move(distance);
 ```
chassis.move(24);
chassis.move(-24);

```
### chassis.turn(angle);
 ```
chassis.turn(90);
chassis.turn(-90);

```
### chassis.swing(angle, bool isLeft);
 ```
chassis.swing(90, false); // swings 90 to the right
chassis.move(-90, true); //swings -90 to the left

```
### chassis.arc(angle, leftMult, rightMult);
 ```
// go to an angle with a certain curvature
chassis.arc(90, 0.3, 0.6);


#### _Contributions welcome. Anything missing? Send in a pull request. Thanks._
Follow  ([@gaelforcev](https://instagram.com/gaelforcev)) on Instagram for updates.

---

