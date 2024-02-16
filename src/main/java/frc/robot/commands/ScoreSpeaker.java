// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team2930.PIDTargetMeasurement;
import frc.lib.team2930.ShootingSolver;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.DrivetrainWrapper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.visualization.GamepieceVisualization;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class ScoreSpeaker extends Command {
  private DrivetrainWrapper drive;
  private Shooter shooter;

  private final LoggedTunableNumber rotationKp =
      new LoggedTunableNumber("RotateToSpeaker/rotationKp", 4.9);
  private final LoggedTunableNumber rotationKd =
      new LoggedTunableNumber("RotateToSpeaker/rotationKd", 0.0);

  private final PIDController rotationController;

  public static final double DEADBAND = 0.1;

  // Creates a new flat moving average filter
  // Average will be taken over the last 20 samples
  private LinearFilter pidLatencyfilter = LinearFilter.movingAverage(10);

  private ArrayList<PIDTargetMeasurement> targetMeasurements =
      new ArrayList<PIDTargetMeasurement>();

  private double pidLatency = 0.0;

  private ShootingSolver solver =
      new ShootingSolver(
          Constants.FieldConstants.getSpeakerTranslation3D(),
          new Translation3d(0, 0, 0),
          new Translation3d(0, -10, 0),
          Constants.ShooterConstants.SHOOTING_SPEED,
          Constants.ShooterConstants.SHOOTING_TIME);

  private BooleanSupplier shootGamepiece;

  private boolean readyToShoot;

  private GamepieceVisualization gamepieceVisualizer = new GamepieceVisualization();

  private Double deadline;

  private double shootDeadline;

  /**
   * Creates a new RotateToSpeaker.
   *
   * @param drive drivetrain subsystem
   * @param shooter shooter subsystem
   * @param shootGamepiece when this command should end
   * @return Command to lock rotation in direction of target
   */
  public ScoreSpeaker(
      DrivetrainWrapper drive, Shooter shooter, BooleanSupplier shootGamepiece, double deadline) {
    this(drive, shooter, shootGamepiece);
    this.deadline = deadline;
  }

  /**
   * Creates a new RotateToSpeaker.
   *
   * @param drive drivetrain subsystem
   * @param shooter shooter subsystem
   * @param shootGamepiece when this command should end
   * @return Command to lock rotation in direction of target
   */
  public ScoreSpeaker(DrivetrainWrapper drive, Shooter shooter, BooleanSupplier shootGamepiece) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.shooter = shooter;
    this.shootGamepiece = shootGamepiece;

    this.rotationController = new PIDController(rotationKp.get(), 0, rotationKd.get());
    setName("ScoreSpeaker");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    shootDeadline = deadline != null ? Timer.getFPGATimestamp() + deadline : Double.MAX_VALUE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var currentTime = Timer.getFPGATimestamp();
    var poseEstimatorPose = drive.getPoseEstimatorPose();
    var currentRot = drive.getPoseEstimatorPose().getRotation();

    var result =
        solver.computeAngles(
            currentTime, poseEstimatorPose, drive.getFieldRelativeVelocities().getTranslation());

    double targetRotation;
    double targetAngularSpeed;
    Rotation2d targetPitch;

    if (result == null) {
      targetRotation = poseEstimatorPose.getRotation().getRadians();
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
      if ((measurement.upDirection && currentRot.getRadians() >= measurement.targetRot.getRadians())
          || (!measurement.upDirection
              && currentRot.getRadians() <= measurement.targetRot.getRadians())) {
        pidLatency = pidLatencyfilter.calculate(Timer.getFPGATimestamp() - measurement.timestamp);
        targetMeasurements.remove(index);
      } else if (Timer.getFPGATimestamp() - measurement.timestamp >= 3.0) {
        targetMeasurements.remove(index);
      }
    }

    Logger.recordOutput("ScoreSpeaker/PIDLatency", pidLatency);

    var rotationalEffort =
        rotationController.calculate(currentRot.getRadians(), targetRotation) + targetAngularSpeed;

    rotationalEffort =
        Math.copySign(
            Math.min(Math.abs(rotationalEffort), drive.getMaxAngularSpeedRadPerSec()),
            rotationalEffort);

    Logger.recordOutput(
        "ScoreSpeaker/targetRotationDegrees",
        Units.radiansToDegrees(targetRotation) <= 360.0
            ? Units.radiansToDegrees(targetRotation)
            : Units.radiansToDegrees(targetRotation) - 360.0);

    Logger.recordOutput(
        "ScoreSpeaker/targetPose",
        new Pose2d(poseEstimatorPose.getTranslation(), Rotation2d.fromRadians(targetRotation)));

    drive.setRotationOverride(rotationalEffort);

    // TODO: remove most of these once we are happy with the command
    Logger.recordOutput("ScoreSpeaker/RotationalEffort", rotationalEffort);
    Logger.recordOutput(
        "ScoreSpeaker/rotationalErrorDegrees",
        Units.radiansToDegrees(rotationController.getPositionError()));
    Logger.recordOutput("ScoreSpeaker/feedForward", targetAngularSpeed);
    Logger.recordOutput(
        "ScoreSpeaker/robotRotationDegrees",
        drive.getPoseEstimatorPose().getRotation().getDegrees());
    Logger.recordOutput("ScoreSpeaker/atSetpoint", rotationController.atSetpoint());

    if (rotationKp.hasChanged(hashCode()) || rotationKd.hasChanged(hashCode())) {
      rotationController.setP(rotationKp.get());
      rotationController.setD(rotationKd.get());
    }

    shooter.setLauncherRPM(Constants.ShooterConstants.SHOOTING_RPM);

    shooter.setPivotPosition(targetPitch);

    rotationController.setTolerance(
        Units.degreesToRadians(
            (Units.feetToMeters(15.0)
                / Math.hypot(
                    Constants.FieldConstants.getSpeakerTranslation().getX()
                        - drive.getPoseEstimatorPose().getX(),
                    Constants.FieldConstants.getSpeakerTranslation().getY()
                        - drive.getPoseEstimatorPose().getY()))));

    if (result != null && !readyToShoot) {
      if (currentTime > deadline) {
        readyToShoot = true;
      } else {
        readyToShoot =
            shootGamepiece.getAsBoolean()
                && shooter.isPivotIsAtTarget()
                && rotationController.atSetpoint()
                && shooter.isAtTargetRPM()
                && shootingPosition();

        if (readyToShoot) {
          solver.startShooting(Timer.getFPGATimestamp());
        }
      }

      Logger.recordOutput("ScoreSpeaker/shooting/IsAtTargetRPM", shooter.isAtTargetRPM());
      Logger.recordOutput(
          "ScoreSpeaker/shooting/RotationControllerAtSetpoint", rotationController.atSetpoint());
      Logger.recordOutput("ScoreSpeaker/shooting/PivotAtTarget", shooter.isPivotIsAtTarget());
      Logger.recordOutput("ScoreSpeaker/shooting/ShootGamepiece", shootGamepiece.getAsBoolean());
      Logger.recordOutput("ScoreSpeaker/shooting/Position", shootingPosition());
      Logger.recordOutput("ScoreSpeaker/shooting/shooting", readyToShoot);

      if (solver.isShooting()) {
        shooter.setKickerPercentOut(Constants.ShooterConstants.Kicker.KICKING_PERCENT_OUT);
      } else if (readyToShoot) {
        shooter.setKickerPercentOut(0.0);
      }
    }

    updateVisualization();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.resetVelocityOverride();
    drive.resetRotationOverride();

    Logger.recordOutput("ScoreSpeaker/targetRotationDegrees", 0.0);
    Logger.recordOutput("ScoreSpeaker/RotationalEffort", 0.0);
    Logger.recordOutput("ScoreSpeaker/rotationalErrorDegrees", 0.0);
    Logger.recordOutput("ScoreSpeaker/feedForward", 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return readyToShoot && !solver.isShooting();
  }

  private boolean shootingPosition() {
    Pose2d reflectedPose =
        Constants.isRedAlliance()
            ? AllianceFlipUtil.mirrorPose2DOverCenterLine(drive.getPoseEstimatorPose())
            : drive.getPoseEstimatorPose();
    // look at constraints: https://www.desmos.com/calculator/dvrwcfwnz8
    // check if shot is legal
    if ((DriverStation.isAutonomous() && reflectedPose.getX() >= 6.2697529792785645)
        || reflectedPose.getX() >= 10.257804870605469) {
      return false;
    }
    // y <= 0.808x + 0.793
    // y >= -0.64x + 6.1
    // y <= 6.103558540344238
    // check if stage is blocking
    if (reflectedPose.getY() <= 0.808 * reflectedPose.getX() + 0.3
        && reflectedPose.getY() >= -0.64 * reflectedPose.getX() + 6.1
        && reflectedPose.getY() <= 6.103558540344238) {
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

    Logger.recordOutput("ShootSpeaker/timer", shootingTimer.get());

    Logger.recordOutput("ShootSpeaker/ShootingRPM", shooter.getRPM());

    GamepieceVisualization.getInstance()
        .updateVisualization(
            drive.getPoseEstimatorPose(),
            drive.getFieldRelativeVelocities().getTranslation(),
            shooter.getPitch(),
            shooter.getRPM(),
            readyToShoot);

    GamepieceVisualization.getInstance().logTraj();
  }
}
