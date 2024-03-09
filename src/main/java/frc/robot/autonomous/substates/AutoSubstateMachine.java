// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.substates;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.StateMachine;
import frc.robot.autonomous.ChoreoHelper;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.commands.shooter.ShooterScoreSpeakerStateMachine;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.visionGamepiece.ProcessedGamepieceData;
import java.util.function.Supplier;

public class AutoSubstateMachine extends StateMachine {
  private final DrivetrainWrapper drive;
  private final Shooter shooter;
  private final EndEffector endEffector;
  private final Intake intake;
  private final Elevator elevator;
  private final Arm arm;
  private final RobotConfig config;
  private final ChoreoTrajectory trajToGamePiece;
  private final ChoreoTrajectory trajToShoot;
  private final Supplier<ProcessedGamepieceData> closestGamepiece;
  private ChoreoHelper choreoHelper;
  public Command scoreSpeaker;
  private IntakeGamepiece intakeCommand;

  /** Creates a new AutoSubstateMachine. */
  public AutoSubstateMachine(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      RobotConfig config,
      Elevator elevator,
      Arm arm,
      String trajToGP,
      String trajToShoot,
      Supplier<ProcessedGamepieceData> closestGamepiece) {
    super(String.format("AutoSub %s / %s", trajToGP, trajToShoot));

    this.drive = drive;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.intake = intake;
    this.elevator = elevator;
    this.arm = arm;
    this.config = config;
    this.trajToGamePiece = Choreo.getTrajectory(trajToGP);
    this.trajToShoot = Choreo.getTrajectory(trajToShoot);
    this.closestGamepiece = closestGamepiece;

    setInitialState(stateWithName("initFollowPathToGamePiece", this::initFollowPathToGamePiece));
  }

  private StateHandler initFollowPathToGamePiece() {
    intakeCommand = new IntakeGamepiece(intake, endEffector, shooter, arm, elevator);
    intakeCommand.schedule();
    choreoHelper =
        new ChoreoHelper(
            timeFromStart(),
            drive.getPoseEstimatorPose(true),
            this.trajToGamePiece,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController());

    return stateWithName("followPathToGamePiece", this::followPathToGamePiece);
  }

  private StateHandler followPathToGamePiece() {
    var chassisSpeeds =
        choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(true), timeFromStart());
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

    scoreSpeaker =
        ShooterScoreSpeakerStateMachine.getAsCommand(drive, shooter, endEffector, intake, 5);

    spawnCommand(
        scoreSpeaker,
        (command) -> {
          drive.resetVelocityOverride();
          return setDone();
        });

    choreoHelper =
        new ChoreoHelper(
            timeFromStart(),
            drive.getPoseEstimatorPose(true),
            this.trajToShoot,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController());

    return stateWithName("followPathToShooter", this::followPathToShooter);
  }

  private StateHandler followPathToShooter() {
    var chassisSpeeds =
        choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(true), timeFromStart());

    if (chassisSpeeds != null) {
      drive.setVelocityOverride(chassisSpeeds);
    } else {
      drive.resetVelocityOverride();
    }

    return null;
  }
}
