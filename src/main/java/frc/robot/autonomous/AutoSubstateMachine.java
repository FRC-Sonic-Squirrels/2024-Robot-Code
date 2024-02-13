// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import frc.lib.team2930.StateMachine;
import frc.robot.DrivetrainWrapper;
import frc.robot.commands.ScoreSpeaker;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.visionGamepiece.ProcessedGamepieceData;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoSubstateMachine extends StateMachine {
  private DrivetrainWrapper drive;
  private Shooter shooter;
  private EndEffector endEffector;
  private Intake intake;
  private RobotConfig config;
  private String trajToGamepieceName;
  private ChoreoTrajectory trajToGamePiece;
  private ChoreoTrajectory trajToShoot;
  private Supplier<ProcessedGamepieceData> closestGamepiece;
  private ChoreoHelper choreoHelper;
  public ScoreSpeaker scoreSpeaker;

  /** Creates a new AutoSubstateMachine. */
  public AutoSubstateMachine(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      RobotConfig config,
      String trajToGP,
      String trajToShoot,
      Supplier<ProcessedGamepieceData> closestGamepiece) {
    this.drive = drive;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.intake = intake;
    this.config = config;
    this.trajToGamepieceName = trajToGP;
    this.trajToGamePiece = Choreo.getTrajectory(trajToGP);
    this.trajToShoot = Choreo.getTrajectory(trajToShoot);
    this.closestGamepiece = closestGamepiece;

    setInitialState(this::initFollowPathToGamePiece);

    Logger.recordOutput("Autonomous/SubstateConstructed", true);
  }

  private StateHandler initFollowPathToGamePiece() {
    choreoHelper =
        new ChoreoHelper(
            timeFromStart(),
            this.trajToGamePiece,
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController(),
            drive.getPoseEstimatorPose());
    return this::followPathToGamePiece;
  }

  private StateHandler followPathToGamePiece() {
    Logger.recordOutput("Autonomous/followGPpathStarted", true);
    Logger.recordOutput("Autonomous/" + trajToGamepieceName + "Started", true);
    Logger.recordOutput("Autonomous/timeFromStart", timeFromStart());
    var chassisSpeeds =
        choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(), timeFromStart());
    if (chassisSpeeds != null) {
      // TODO: Check for note in intake.
      drive.setVelocityOverride(chassisSpeeds);
      return null;
    }

    if (!intake.getBeamBreak()) {
      Logger.recordOutput("Autonomous/stopped", true);
      return setStopped();
    }

    Logger.recordOutput("Autonomous/followPathToShooterStarted", true);

    scoreSpeaker = new ScoreSpeaker(drive, shooter, () -> true);
    scoreSpeaker.schedule();

    choreoHelper =
        new ChoreoHelper(
            timeFromStart(),
            this.trajToShoot,
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController(),
            drive.getPoseEstimatorPose());

    return this::followPathToShooter;
  }

  private StateHandler followPathToShooter() {
    Logger.recordOutput("Autonomous/time", timeFromStart());
    scoreSpeaker.updateVisualization();
    if (scoreSpeaker.isFinished()) {
      Logger.recordOutput("Autonomous/ShooterDone", true);
      return setDone();
    }

    var chassisSpeeds =
        choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(), timeFromStart());
    if (chassisSpeeds != null) {
      drive.setVelocityOverride(chassisSpeeds);
      return null;
    }

    Logger.recordOutput("Autonomous/stopped", true);
    drive.resetVelocityOverride();
    return null;
  }
}
