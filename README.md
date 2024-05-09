# 2024-Robot-Code
## Introduction

Competition code for team 2930, The Sonic Squirrels, 2024 FRC Crescendo robot "Maestro".

Won innovation in controls award at district event and autonomous award at PNW regional championship.

## Features
- Full-field vision using 2 April Tag cameras and 2 Orange Pis
- Multi-tag PNP support
--------------
- Mid-game automated alignment to amp
- Mid-game automated alignment to trap
--------------
- Autonomous state machines for better logic organization
  ![State Machine diagram](https://github.com/FRC-Sonic-Squirrels/2024-Robot-Code/assets/92405837/a0066658-8b5a-4141-ad7e-9b6d65bcf23c)
- Ability to dynamically adjust to next game piece in auto if desired one is missing
  [footage of this in action](https://www.thebluealliance.com/match/2024mil_qm15)
  ![Auto Decision Tree](https://github.com/FRC-Sonic-Squirrels/2024-Robot-Code/assets/92405837/ae6d1e77-4f57-469c-a7da-7b00edcec668)
- Custom path following using Choreo [Choreo](https://github.com/SleipnirGroup/Choreo)
  ![ChoreoHelper](https://github.com/FRC-Sonic-Squirrels/2024-Robot-Code/assets/92405837/d41b2ca9-296f-423a-b696-3c95fa36031f)
- 15 autos with multiple permutations of each auto
- Neural network "AI" game piece vision using Photon vision for dynamic path adjustment
  [footage of this in action](https://www.thebluealliance.com/match/2024pncmp_qm91)
  [Photon vision object detection](https://docs.photonvision.org/en/latest/docs/objectDetection/about-object-detection.html)
--------------
- Simulated vision  (note: does not support multi-tag)
- Simulated swerve (per 3061-lib)
- Simulated Elevator, Arm, and Shooter
--------------
- Logging and replay via [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/README.md)

## Notable Files
- [`autonomous`](/src/main/java/frc/robot/autonomous) - Autonomous

- [`CommandComposer.java`](/src/main/java/frc/robot/CommandComposer.java) - Driver Assist functions

- [`vision`](/src/main/java/frc/robot/subsystems/vision) - Vision system

- [`ShootingSolver.java`](/src/main/java/frc/lib/team2930/ShootingSolver.java) - Shooting Solver

## Credits & References

- 3061-lib for the swerve library and base advantage kit structuring [3061 Lib](https://github.com/HuskieRobotics/3061-lib)
* MK4/MK4i code initially from Team 364's [BaseFalconSwerve](https://github.com/Team364/BaseFalconSwerve)
* general AdvantageKit logging code, AdvantageKit-enabled Gyro classes, swerve module simulation and drive characterization from Mechanical Advantage's [SwerveDevelopment](https://github.com/Mechanical-Advantage/SwerveDevelopment)
* Setting up Spotless code linting [WPILib Spotless setup](https://docs.wpilib.org/en/latest/docs/software/advanced-gradlerio/code-formatting.html#spotless)
* Sleipnir Group Choreo [Choreo](https://github.com/SleipnirGroup/Choreo)
