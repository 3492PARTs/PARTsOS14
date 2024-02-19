# Tuning the SysID (2024)
- [Tuning the SysID (2024)](#tuning-the-sysid-2024)
  - [Preface](#preface)
  - [Creating the routine.](#creating-the-routine)
  - [Running the Routine](#running-the-routine)
  - [Getting the Data](#getting-the-data)
    - [Data Log Tool](#data-log-tool)
    - [SysID Tool](#sysid-tool)
  - [Analyzing the Data](#analyzing-the-data)
  - [Applying the Data](#applying-the-data)
  - [Troubleshooting](#troubleshooting)
    - [2024 Specific Problems](#2024-specific-problems)
      - [REV - ```motor.get()``` returning Zero](#rev---motorget-returning-zero)
    - [Quasistatic test trimming removed all data.](#quasistatic-test-trimming-removed-all-data)
## Preface
You want to start with having something like a feedforward system set up.\
Make sure to bind your movement buttons to physical buttons or use an auto routine.
## Creating the routine.
Follow the WIPLib guide for this. [Creating an Identification Routine](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html)\
This is a structure of how the routine is made.\
You start by creating a new SysIdRoutine with the args being ```SysIdRoutine.Config()``` and ```SysIdRoutine.Mechanism()```.\
Keeping the ```SysIdRoutine.Config()``` blank will be desirable in most cases.
However, for the ```SysIdRoutine.Mechanism()``` you will need to supply the argument ```log -> {}``` to actually log the motor(s) for each frame.\
Inside of ```log -> {}``` you can specify each motor with ```log.motor()```.\
Under ```log.motor()``` specifying ```.voltage()```, ```.linearPosition()```, and ```.linearVelocity()``` is needed to properly log the motors each frame.
```java
public SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
      (voltage) -> this.driveMotorVolts(voltage.in(Volts)),

      log -> {
        // Record a frame for the motor.
        log.motor("motor")
            .voltage(m_appliedVoltage.mut_replace(theVoltage, Volts))
            .angularPosition(m_angle.mut_replace(thePosition, Rotations))
            .angularVelocity(m_velocity.mut_replace(theVelocity, RotationsPerSecond));
      },
      this
    )
  );
```
## Running the Routine
Follow the WPILib guide for this. [Running the Identification Routine](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/running-routine.html).
## Getting the Data
WPILib document: [Loading Data](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/loading-data.html).\
To get the datalog for SysID you start by opening the *Data Log Tool* found in your WPILib installation.
### Data Log Tool
Input your team number or IP into the first box.\
Next keep the username 'lvuser' and the password blank if you're using a roboRIO.\
To select the latest log, usually the bottom most log is the latest log, but you can check in the RIOLog for the log file name.\
After you select the log you want, save it to a folder.
### SysID Tool
Press the 'Open data log file...' button and find the log you want to use that you saved earlier.\
Next, drag 'test-state' into your Data Selector window.\
Afterwords, drag the corresponding values into the Data Selector and press Load.\
If you got an error here then refer to the Troubleshooting section below.
## Analyzing the Data
Looking in the Analyzer should give you the correct values that you would need for for feewforward and PID, etc.\
If you get NaN (Not a Number) errors then refer to the troubleshooting section below.
## Applying the Data
To apply your data to your code, adjust the PID, and feedforward values with the values from the SysID tool.
## Troubleshooting
### 2024 Specific Problems
#### REV - ```motor.get()``` returning Zero
REV is bugged with this years SysID tool.\
IF you are using ```motor.get()``` with ```RobotController.getBatteryVoltage()```, it will always return zero.\
To fix this replace your ```motor.get()``` with ```motor.getAppliedOutput()``` and replace ```RobotController.getBatteryVoltage()``` with ```motor.getBusVoltage()```.
### Quasistatic test trimming removed all data.
You will get this error if your data that has been logged is not there, or messed up.\
This could be for multipule reasons spanning from bugs to incorrect testing.\
Generally this is an issue with the test or code.\
If the issue is with the code, then it is advised to look carefully through your ```SysIdRoutine``` code.