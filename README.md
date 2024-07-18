# PARTsOS14 ![Progress](https://progress-bar.dev/98)
Source code for the PARTs 2024 robot.

## Code
We are using the latest [WPILib 2024 Release](https://github.com/wpilibsuite/allwpilib/releases/tag/latest).\
We are programming our robot with Java.
The code is the brain of our bot, hosting all of our amazing autos, subsystems, and commands that really make our robot pop!
## Drivetrain
The drivetrain uses four CANSparkMaxes in a Leader-Follower configuration like tank drive.\
Using treads provides us with the ability to stop and not be moved by any foe.\
The treads grip the ground enough to prevent any tipping or moving.\
The chassis is heavy enough to shrug off quite a few hits.
## Arm
The arm is 1DoF with a singular pivot point at the base of the robot.\
The arm is powered via two CANSparkMaxes configured with a 64:1 gearbox.\
It holds itself up with the power of PID.
## Intake
The intake is a configuration of rollers connected via two conveyor belts on each side.\
They are powered by one Talon SRX, just powerful enough to pickup notes with ease and agility on the field.
## Shooter
The shooter is a configuration of much larger rollers designed to shoot out the notes at varied speeds and angles.\
It is powered by two Talon SRXs and packs a punch on the field.
## Climber
Powered by two motors with a 5.87 to 1 gear ratio, we can hang and even buddy climb!
## Controllers
[Visit our page about our controllers!](./docs/controller/ControllerBindings.md)
