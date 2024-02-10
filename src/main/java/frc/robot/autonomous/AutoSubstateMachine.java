// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.team2930.StateMachine;
import frc.robot.DrivetrainWrapper;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class AutoSubstateMachine extends StateMachine {
  private DrivetrainWrapper drive;
  private Shooter shooter;
  private EndEffector endEffector;
  private Intake intake;
  private ChoreoHelper choreoHelper;
  private RobotConfig config;
  private String trajToGP;
  private String trajToShoot;
  private Timer runTime = new Timer();
  private double distToIntakeGP = 1.5;

  /** Creates a new AutoSubstateMachine. */
  public AutoSubstateMachine(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      RobotConfig config,
      String trajToGP,
      String trajToShoot,
      DoubleSupplier distToGamepiece) {
    this.drive = drive;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.intake = intake;
    this.config = config;
    this.trajToGP = trajToGP;
    this.trajToShoot = trajToShoot;

    setInitialState(followGPpathInit());
  }

  private StateHandler followGPpathInit() {
    choreoHelper =
        new ChoreoHelper(
            trajToGP,
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController(),
            drive.getPoseEstimatorPose());
    runTime.start();
    return this.followGPpath();
  }

  private StateHandler followGPpath() {
    drive.setVelocity(
        choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(), runTime.get()));

    return null;
    // return distance.get()<=distToIntakeGP ? :null;
  }

  // private StateHandler intakeGamepiece(){
  //   drive.setVelocity(null);
  // }

}
