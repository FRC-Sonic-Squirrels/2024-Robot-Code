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
  protected final DrivetrainWrapper drive;
  protected final Shooter shooter;
  protected final EndEffector endEffector;
  protected final Intake intake;
  protected final Elevator elevator;
  protected final Arm arm;
  protected final RobotConfig config;
  protected final ChoreoTrajectory trajToShoot;
  protected final Supplier<ProcessedGamepieceData> closestGamepiece;
  protected ChoreoHelper choreoHelper;
  protected DriveToGamepieceHelper driveToGamepieceHelper;
  public Command scoreSpeaker;
  protected IntakeGamepiece intakeCommand;
  protected Translation2d gamepieceTranslation;
  private Translation2d lastSeenGamepiece;

  private LoggedTunableNumber distToBeginDriveToGamepiece =
      new LoggedTunableNumber("AutoSubstateMachine/distToBeginDriveToGamepiece", 2.0);

  /** Creates a new AutoSubstateMachine. */
  protected AutoSubstateMachine(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      RobotConfig config,
      Elevator elevator,
      Arm arm,
      ChoreoTrajectory trajToShoot,
      Supplier<ProcessedGamepieceData> closestGamepiece,
      Translation2d gamepieceTranslation) {
    super(String.format("AutoSub %s", trajToShoot));

    this.drive = drive;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.intake = intake;
    this.elevator = elevator;
    this.arm = arm;
    this.config = config;
    this.trajToShoot = trajToShoot;
    this.closestGamepiece = closestGamepiece;
    this.gamepieceTranslation = gamepieceTranslation;
  }

  protected StateHandler visionPickupGamepiece() {
    if (closestGamepiece.get() != null)
      lastSeenGamepiece = closestGamepiece.get().globalPose.getTranslation();

    ChassisSpeeds speeds =
        driveToGamepieceHelper.calculateChassisSpeeds(
            lastSeenGamepiece, drive.getPoseEstimatorPose(true));

    if (speeds != null) drive.setVelocityOverride(speeds);

    if (!endEffector.noteInEndEffector()) {
      if (driveToGamepieceHelper.isAtTarget()) {
        drive.resetVelocityOverride();
        return setStopped();
      }
      return null;
    }
    drive.resetVelocityOverride();
    return stateWithName("prepFollowPathToShooting", this::prepFollowPathToShooting);
  }

  protected StateHandler prepFollowPathToShooting() {
    intakeCommand.cancel();

    scoreSpeaker =
        ShooterScoreSpeakerStateMachine.getAsCommand(drive, shooter, endEffector, intake, 5);

    spawnCommand(
        scoreSpeaker,
        (command) -> {
          drive.resetVelocityOverride();
          return setDone();
        });

    if (trajToShoot != null)
      choreoHelper =
          new ChoreoHelper(
              timeFromStart(),
              drive.getPoseEstimatorPose(true),
              this.trajToShoot,
              config.getAutoTranslationPidController(),
              config.getAutoTranslationPidController(),
              config.getAutoThetaPidController());

    return stateWithName("followPathToShooter", this::followPathToShooting);
  }

  private StateHandler followPathToShooting() {
    ChassisSpeeds chassisSpeeds = null;
    if (trajToShoot != null)
      chassisSpeeds =
          choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(true), timeFromStart());

    if (chassisSpeeds != null) {
      drive.setVelocityOverride(chassisSpeeds);
    } else {
      drive.resetVelocityOverride();
    }

    return null;
  }

  protected boolean useVisionForGamepiece() {
    if (closestGamepiece.get() == null) {
      return false;
    }
    return closestGamepiece.get().getDistance(drive.getPoseEstimatorPose(true)).in(Units.Meters)
            <= distToBeginDriveToGamepiece.get()
        && closestGamepiece
                .get()
                .getDistance(new Pose2d(gamepieceTranslation, new Rotation2d()))
                .in(Units.Meters)
            <= Constants.FieldConstants.Gamepieces.NOTE_TOLERANCE.in(Units.Meters);
  }
}
