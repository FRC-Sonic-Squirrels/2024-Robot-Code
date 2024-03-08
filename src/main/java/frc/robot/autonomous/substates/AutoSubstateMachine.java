// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.substates;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.StateMachine;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.autonomous.ChoreoHelper;
import frc.robot.autonomous.DriveToGamepieceHelper;
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

public abstract class AutoSubstateMachine extends StateMachine {
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
  private final Translation2d gamepeiceTranslation;
  private ChoreoHelper choreoHelper;
  private DriveToGamepieceHelper driveToGamepieceHelper;
  public Command scoreSpeaker;
  private IntakeGamepiece intakeCommand;
  private boolean up;

  private LoggedTunableNumber distToBeginDriveToGamepiece =
      new LoggedTunableNumber("AutoSubstateMachine/distToBeginDriveToGamepiece", 2.0);

  private Trigger robotStopped;

  /** Creates a new AutoSubstateMachine. */
  public AutoSubstateMachine(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      RobotConfig config,
      Elevator elevator,
      Arm arm,
      ChoreoTrajectory trajToGP,
      ChoreoTrajectory trajToShoot,
      Supplier<ProcessedGamepieceData> closestGamepiece,
      Translation2d gamepiecePose) {
    super(String.format("AutoSub %s / %s", trajToGP, trajToShoot));

    this.drive = drive;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.intake = intake;
    this.elevator = elevator;
    this.arm = arm;
    this.config = config;
    this.trajToGamePiece = trajToGP;
    this.trajToShoot = trajToShoot;
    this.closestGamepiece = closestGamepiece;
    this.gamepeiceTranslation = gamepiecePose;

    robotStopped =
        new Trigger(
                () ->
                    (drive.getFieldRelativeVelocities().getX() <= 0.01
                        && drive.getFieldRelativeVelocities().getY() <= 0.01
                        && drive.getFieldRelativeVelocities().getRotation().getRadians() <= 0.01))
            .debounce(0.2);

    setInitialState(stateWithName("initFollowPathToGamePiece", this::initFollowPathToGamePiece));
  }

  private StateHandler initFollowPathToGamePiece() {
    intakeCommand = new IntakeGamepiece(intake, endEffector, shooter, arm, elevator);
    intakeCommand.schedule();
    if (trajToGamePiece != null)
      choreoHelper =
          new ChoreoHelper(
              timeFromStart(),
              drive.getPoseEstimatorPose(),
              this.trajToGamePiece,
              config.getAutoTranslationPidController(),
              config.getAutoTranslationPidController(),
              config.getAutoThetaPidController());

    driveToGamepieceHelper = new DriveToGamepieceHelper();

    return stateWithName("followPathToGamePiece", this::pickupGamepiece);
  }

  private StateHandler pickupGamepiece() {
    Translation2d targetTranslation = gamepeiceTranslation;

    if (useVisionForGamepiece()) {
      targetTranslation = closestGamepiece.get().globalPose.getTranslation();
    } else {
      if (gamepeiceTranslation != null) targetTranslation = gamepeiceTranslation;
    }

    ChassisSpeeds speeds =
        driveToGamepieceHelper.calculateChassisSpeeds(
            targetTranslation, drive.getPoseEstimatorPose());
    if (!useVisionForGamepiece() && trajToGamePiece != null) {
      speeds = choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(), timeFromStart());
    }

    if (!endEffector.noteInEndEffector()) {
      drive.setVelocityOverride(speeds);
      if (robotStopped.getAsBoolean()) {
        drive.resetVelocityOverride();
        return setStopped();
      }
      return null;
    }
    return stateWithName("prepFollowPathToShooting", this::prepFollowPathToShooting);
  }

  private StateHandler prepFollowPathToShooting() {
    intakeCommand.cancel();

    scoreSpeaker =
        ShooterScoreSpeakerStateMachine.getAsCommand(drive, shooter, endEffector, intake, 5);

    spawnCommand(
        scoreSpeaker,
        (command) -> {
          drive.resetVelocityOverride();
          return setDone();
        });

    if (trajToShoot != null) {
      choreoHelper =
          new ChoreoHelper(
              timeFromStart(),
              drive.getPoseEstimatorPose(),
              this.trajToShoot,
              config.getAutoTranslationPidController(),
              config.getAutoTranslationPidController(),
              config.getAutoThetaPidController());
    }

    return stateWithName("followPathToShooter", this::followPathToShooting);
  }

  private StateHandler followPathToShooting() {
    ChassisSpeeds chassisSpeeds;
    if (trajToShoot != null) {
      chassisSpeeds =
          choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(), timeFromStart());
    } else {
      chassisSpeeds = new ChassisSpeeds();
    }

    if (chassisSpeeds != null) {
      drive.setVelocityOverride(chassisSpeeds);
    } else {
      drive.resetVelocityOverride();
    }

    return null;
  }

  private boolean useVisionForGamepiece() {
    return closestGamepiece.get().getDistance(drive.getPoseEstimatorPose()).in(Units.Meters)
            <= distToBeginDriveToGamepiece.get()
        && closestGamepiece
                .get()
                .getDistance(new Pose2d(gamepeiceTranslation, new Rotation2d()))
                .in(Units.Meters)
            <= Constants.FieldConstants.Gamepieces.NOTE_TOLERANCE.in(Units.Meters);
  }
}
