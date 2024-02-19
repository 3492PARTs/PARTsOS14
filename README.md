# PARTsOS14 ![Progress](https://progress-bar.dev/34)
Source code for the PARTs 2024 robot.

## Code
We are using the [WPILib 2024.2.1 Release](https://github.com/wpilibsuite/allwpilib/releases/tag/v2024.2.1).\
We are coding in the Java programming language with the OpenJDK that WPILib comes bundled with.
## Drivetrain
The drivetrain uses four CANSparkMaxes in a Leader-Follower configuration like tank drive.
## Arm
The arm is 1DoF with a singular pivot point at the base of the robot.\
The arm is powered via two CANSparkMaxes configured with a 64:1 gearbox.
## Intake
The intake is a configuration of rollers connected via two conveyor belts on each side.\
They are powered by one Talon SRX.
## Shooter
The shooter is a configuration of much larger rollers designed to shoot out the notes at high speeds.\
It is powered by two Talon SRXs.
