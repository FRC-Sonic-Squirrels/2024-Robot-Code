// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.choreo.lib.ChoreoTrajectory;
import frc.lib.team2930.StateMachine;
import frc.robot.DrivetrainWrapper;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Drivetrain;

public class AutoSubstateMachine extends StateMachine {
  private Drivetrain drive;
  private Shooter shooter;
  private EndEffector endEffector;
  private Intake intake;
  private ChoreoHelper choreoHelper;
  private DrivetrainWrapper driveWrapper;
  private RobotConfig config;
  private ChoreoTrajectory traj;

  /** Creates a new AutoSubstateMachine. */
  public AutoSubstateMachine(
      Drivetrain drive,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      RobotConfig config,
      ChoreoTrajectory traj) {
    this.drive = drive;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.intake = intake;
    this.config = config;
    this.traj = traj;
  }
}
