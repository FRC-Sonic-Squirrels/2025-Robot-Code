# 2025-Robot-Code
## Introduction
Competition code for team 2930, The Sonic Squirrels, 2025 FRC Reefscape robot "Holocentrus".

Won autonomous award at GPK and innovation in controls award at SunDome.

---------------------------------------------------------------------------

## Key Features
-Autonomous state machine system.

-Periodically controlled intake and endeffector subsystems.

-Automatic collision detection system for smooth motions on the robot.

--------------------------------------------------------------------------

## Autonomous
-State machine system that goes through the actions of intaking and scoring coral as well as pathfinding.

-Uses over 100 tiny choreo paths to construct long strings of autos for flexibility and adaptibility in auto selection.

--------------------------------------------------------------------------

## Periodic Subsystems
-Both the intake and endeffector subsystems run through periodic checks to detemine what action it should be taking.

-Using constants throughout the robot that are changed when certain commands are called, the state of these subsystems is able to be altered.

--------------------------------------------------------------------------

## Collision Detection
-By segmenting the robot into sections based on the angle of the arm and the height of the elevator, when movements to these mechanisms are called it is able to adjust to travel through a path that does not result in a collision.

-By doing this, the robot does not need to worry about colliding into itself to perform some of its different scoring motions.

---------------------------------------------------------------------------

## Credits & References

- 3061-lib for the swerve library and base advantage kit structuring [3061 Lib](https://github.com/HuskieRobotics/3061-lib)
* MK4/MK4i code initially from Team 364's [BaseFalconSwerve](https://github.com/Team364/BaseFalconSwerve)
* general AdvantageKit logging code, AdvantageKit-enabled Gyro classes, swerve module simulation and drive characterization from Mechanical Advantage's [SwerveDevelopment](https://github.com/Mechanical-Advantage/SwerveDevelopment)
* Setting up Spotless code linting [WPILib Spotless setup](https://docs.wpilib.org/en/latest/docs/software/advanced-gradlerio/code-formatting.html#spotless)
* Sleipnir Group Choreo [Choreo](https://github.com/SleipnirGroup/Choreo)
