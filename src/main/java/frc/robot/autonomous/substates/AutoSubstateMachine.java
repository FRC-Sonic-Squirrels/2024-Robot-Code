// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.substates;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import frc.lib.team2930.StateMachine;
import frc.robot.autonomous.ChoreoHelper;
import frc.robot.commands.ScoreSpeaker;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
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
  private IntakeGamepiece intakeCommand;

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
  }

  private StateHandler initFollowPathToGamePiece() {
    intakeCommand = new IntakeGamepiece(intake, endEffector, shooter);
    intakeCommand.schedule();
    choreoHelper =
        new ChoreoHelper(
            timeFromStart(),
            drive.getPoseEstimatorPose(),
            this.trajToGamePiece,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController());
    return this::followPathToGamePiece;
  }

  private StateHandler followPathToGamePiece() {
    Logger.recordOutput("Autonomous/" + trajToGamepieceName + "Started", true);
    var chassisSpeeds =
        choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(), timeFromStart());
    if (chassisSpeeds != null) {
      // TODO: Check for note in intake.
      drive.setVelocityOverride(chassisSpeeds);
      return null;
    }

    // if (!intake.getBeamBreak()) {
    //   Logger.recordOutput("Autonomous/gamepieceNotRecieved", true);
    //   return setStopped();
    // }

    intakeCommand.cancel();

    scoreSpeaker = new ScoreSpeaker(drive, shooter, endEffector, () -> true);
    scoreSpeaker.schedule();

    choreoHelper =
        new ChoreoHelper(
            timeFromStart(),
            drive.getPoseEstimatorPose(),
            this.trajToShoot,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController());

    return this::followPathToShooter;
  }

  private StateHandler followPathToShooter() {
    scoreSpeaker.updateVisualization();
    if (scoreSpeaker.isFinished()) {
      return setDone();
    }

    var chassisSpeeds =
        choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(), timeFromStart());
    if (chassisSpeeds != null) {
      drive.setVelocityOverride(chassisSpeeds);
      return null;
    }

    drive.resetVelocityOverride();
    return null;
  }
}
