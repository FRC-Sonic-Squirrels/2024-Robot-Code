// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.visualization.GamepieceVisualization;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class ScoreSpeaker extends Command {
  private static final String ROOT_TABLE = "ScoreSpeaker";

  private static final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  private static final LoggerGroup logGroup = new LoggerGroup(ROOT_TABLE);
  private static final LoggerEntry log_gamepieceLoaded = logGroup.build("gamepieceLoaded");
  private static final LoggerEntry log_PIDLatency = logGroup.build("PIDLatency");
  private static final LoggerEntry log_targetRotationDegrees =
      logGroup.build("targetRotationDegrees");
  private static final LoggerEntry log_targetPose = logGroup.build("targetPose");
  private static final LoggerEntry log_RotationalEffort = logGroup.build("RotationalEffort");
  private static final LoggerEntry log_RotationalErrorDegrees =
      logGroup.build("RotationalErrorDegrees");
  private static final LoggerEntry log_feedForward = logGroup.build("feedForward");
  private static final LoggerEntry log_RobotRotationDegrees =
      logGroup.build("RobotRotationDegrees");
  private static final LoggerEntry log_atSetpoint = logGroup.build("atSetpoint");
  private static final LoggerEntry log_timer = logGroup.build("timer");
  private static final LoggerEntry log_ShootingRPM = logGroup.build("ShootingRPM");

  private static final LoggerGroup logGroupShooting = logGroup.subgroup("shooting");
  private static final LoggerEntry log_IsAtTargetRPM = logGroupShooting.build("IsAtTargetRPM");
  private static final LoggerEntry log_RotationControllerAtSetpoint =
      logGroupShooting.build("RotationControllerAtSetpoint");
  private static final LoggerEntry log_PivotAtTarget = logGroupShooting.build("PivotAtTarget");
  private static final LoggerEntry log_ShootGamepiece = logGroupShooting.build("ShootGamepiece");
  private static final LoggerEntry log_Position = logGroupShooting.build("Position");
  private static final LoggerEntry log_shooting = logGroupShooting.build("shooting");

  private static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);
  private static final LoggedTunableNumber rotationKp = group.build("rotationKp", 4.9);
  private static final LoggedTunableNumber rotationKd = group.build("rotationKd", 0.0);
  private static final LoggedTunableNumber tunableVoltage = group.build("tunableVoltage", 0.5);

  private final DrivetrainWrapper drive;
  private final Shooter shooter;
  private final EndEffector endEffector;
  private final PIDController rotationController;

  // Creates a new flat moving average filter
  // Average will be taken over the last 20 samples
  private final LinearFilter pidLatencyfilter = LinearFilter.movingAverage(10);
  private final List<PIDTargetMeasurement> targetMeasurements = new ArrayList<>();

  private double pidLatency = 0.0;

  private final ShootingSolver solver =
      new ShootingSolver(
          Constants.FieldConstants.getSpeakerTranslation3D(),
          new Translation3d(0, 0, 0),
          new Translation3d(0, -10, 0),
          Constants.ShooterConstants.SHOOTING_SPEED,
          Constants.ShooterConstants.SHOOTING_TIME,
          false);

  private final BooleanSupplier shootGamepiece;

  private boolean readyToShoot;

  private Double deadline;

  private double shootDeadline;

  private boolean prevNoteInShoot = false;

  private boolean gamepieceLoaded = false;

  /**
   * Creates a new RotateToSpeaker.
   *
   * @param drive drivetrain subsystem
   * @param shooter shooter subsystem
   * @param shootGamepiece when this command should end
   */
  public ScoreSpeaker(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      BooleanSupplier shootGamepiece,
      double deadline) {
    this(drive, shooter, endEffector, shootGamepiece);
    this.deadline = deadline;
  }

  /**
   * Creates a new RotateToSpeaker.
   *
   * @param drive drivetrain subsystem
   * @param shooter shooter subsystem
   * @param shootGamepiece when this command should end
   */
  public ScoreSpeaker(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      BooleanSupplier shootGamepiece) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.shooter = shooter;
    this.shootGamepiece = shootGamepiece;
    this.endEffector = endEffector;

    addRequirements(shooter, endEffector);

    this.rotationController = new PIDController(rotationKp.get(), 0, rotationKd.get());
    setName("ScoreSpeaker");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gamepieceLoaded = false;
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    shootDeadline = deadline != null ? Timer.getFPGATimestamp() + deadline : Double.MAX_VALUE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try (var ignored = timing.start()) {
      if (!shooter.noteInShooter()) {
        shooter.markStartOfNoteLoading();
        shooter.setKickerPercentOut(0.1 * 1.42);

        endEffector.markStartOfNoteDropping();
        endEffector.setPercentOut(0.1);
      } else if (!prevNoteInShoot) {
        shooter.setKickerPercentOut(0.0);
        endEffector.setPercentOut(0.0);
        gamepieceLoaded = true;
      }

      log_gamepieceLoaded.info(gamepieceLoaded);

      prevNoteInShoot = shooter.noteInShooter();

      var currentTime = Timer.getFPGATimestamp();
      var poseEstimatorPose = drive.getPoseEstimatorPose(false);
      var currentRot = drive.getRotationGyroOnly();

      var result =
          solver.computeAngles(
              currentTime,
              new Pose2d(poseEstimatorPose.getTranslation(), drive.getRotationGyroOnly()),
              drive.getFieldRelativeVelocities().getTranslation());

      double targetRotation;
      double targetAngularSpeed;
      Rotation2d targetPitch;

      if (result == null) {
        targetRotation = currentRot.getRadians();
        targetAngularSpeed = 0.0;
        targetPitch = shooter.getPitch();
      } else {
        targetRotation = result.heading().getRadians();
        targetAngularSpeed = result.rotationSpeed();
        targetPitch = result.pitch();
      }

      targetMeasurements.add(
          new PIDTargetMeasurement(
              currentTime, targetRotation, currentRot.getRadians() <= targetRotation));

      for (int index = 0; index < targetMeasurements.size(); index++) {
        PIDTargetMeasurement measurement = targetMeasurements.get(index);
        if ((measurement.upDirection
                && currentRot.getRadians() >= measurement.targetRot.getRadians())
            || (!measurement.upDirection
                && currentRot.getRadians() <= measurement.targetRot.getRadians())) {
          pidLatency = pidLatencyfilter.calculate(Timer.getFPGATimestamp() - measurement.timestamp);
          targetMeasurements.remove(index);
        } else if (Timer.getFPGATimestamp() - measurement.timestamp >= 3.0) {
          targetMeasurements.remove(index);
        }
      }

      log_PIDLatency.info(pidLatency);

      var rotationalEffort =
          rotationController.calculate(currentRot.getRadians(), targetRotation)
              + targetAngularSpeed;

      rotationalEffort =
          Math.copySign(
              Math.min(Math.abs(rotationalEffort), drive.getMaxAngularSpeedRadPerSec()),
              rotationalEffort);

      double targetRotationDegrees = Math.toDegrees(targetRotation);
      if (targetRotationDegrees > 360) targetRotationDegrees -= 360;
      if (targetRotationDegrees < -360) targetRotationDegrees += 360;

      log_targetRotationDegrees.info(targetRotationDegrees);
      log_targetPose.info(
          new Pose2d(poseEstimatorPose.getTranslation(), Rotation2d.fromRadians(targetRotation)));

      // drive.setRotationOverride(rotationalEffort);

      // TODO: remove most of these once we are happy with the command
      log_RotationalEffort.info(rotationalEffort);
      log_RotationalErrorDegrees.info(Math.toDegrees(rotationController.getPositionError()));
      log_feedForward.info(targetAngularSpeed);
      log_RobotRotationDegrees.info(drive.getRotationGyroOnly().getDegrees());
      log_atSetpoint.info(rotationController.atSetpoint());

      if (rotationKp.hasChanged(hashCode()) || rotationKd.hasChanged(hashCode())) {
        rotationController.setP(rotationKp.get());
        rotationController.setD(rotationKd.get());
      }

      shooter.markStartOfNoteShooting();
      shooter.setLauncherVoltage(tunableVoltage.get());

      shooter.setPivotPosition(
          shooter.noteInShooter()
              ? targetPitch
              : Constants.ShooterConstants.Pivot.LOADING_POSITION);

      var speakerDistance =
          Constants.FieldConstants.getDistanceToSpeaker(drive.getPoseEstimatorPose(false));
      rotationController.setTolerance(
          Math.toDegrees(Units.Feet.of(15.0).in(Units.Meters) / speakerDistance.in(Units.Meters)));

      if (result != null && !readyToShoot) {
        if (currentTime > shootDeadline) {
          readyToShoot = true;
        } else {
          readyToShoot =
              // shootGamepiece.getAsBoolean()
              shooter.isPivotIsAtTarget()
          // && rotationController.atSetpoint()
          // && shooter.isAtTargetRPM()
          // && shootingPosition()
          ;

          if (readyToShoot) {
            solver.startShooting(Timer.getFPGATimestamp());
          }
        }

        log_IsAtTargetRPM.info(shooter.isAtTargetRPM());
        log_RotationControllerAtSetpoint.info(rotationController.atSetpoint());
        log_PivotAtTarget.info(shooter.isPivotIsAtTarget());
        log_ShootGamepiece.info(shootGamepiece.getAsBoolean());
        log_Position.info(shootingPosition());
        log_shooting.info(readyToShoot);

        if (gamepieceLoaded) {
          if (solver.isShooting()) {
            shooter.setKickerPercentOut(Constants.ShooterConstants.Kicker.KICKING_PERCENT_OUT);
          } else if (readyToShoot) {
            shooter.setKickerPercentOut(0.0);
          }
        }
      }

      updateVisualization();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.resetVelocityOverride();
    drive.resetRotationOverride();

    shooter.setKickerPercentOut(0.0);
    shooter.setLauncherVoltage(0.0);

    log_targetRotationDegrees.info(0.0);
    log_RotationalEffort.info(0.0);
    log_RotationalErrorDegrees.info(0.0);
    log_feedForward.info(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return readyToShoot && !solver.isShooting();
    return false;
  }

  private boolean shootingPosition() {
    Pose2d reflectedPose = AllianceFlipUtil.flipPoseForAlliance(drive.getPoseEstimatorPose(false));
    double x = reflectedPose.getX();
    double y = reflectedPose.getY();

    // look at constraints: https://www.desmos.com/calculator/dvrwcfwnz8
    // check if shot is legal
    if ((DriverStation.isAutonomous() && x >= 6.2697529792785645) || x >= 10.257804870605469) {
      return false;
    }
    // y <= 0.808x + 0.793
    // y >= -0.64x + 6.1
    // y <= 6.103558540344238
    // check if stage is blocking
    if (y <= 0.808 * x + 0.3 && y >= -0.64 * x + 6.1 && y <= 6.103558540344238) {
      return false;
    }
    return true;
  }

  boolean prevIsShooting = false;

  boolean isShot = false;

  boolean isShotPrev = false;

  boolean shotStart = false;

  Timer shootingTimer = new Timer();

  Pose2d robotPoseOfShot = new Pose2d();

  public void updateVisualization() {
    log_timer.info(shootingTimer.get());
    log_ShootingRPM.info(shooter.getRPM());

    // GamepieceVisualization.getInstance()
    //     .updateVisualization(
    //         drive.getPoseEstimatorPose(),
    //         drive.getFieldRelativeVelocities().getTranslation(),
    //         shooter.getPitch(),
    //         shooter.getRPM(),
    //         readyToShoot);

    GamepieceVisualization.getInstance().logTraj();
  }
}
