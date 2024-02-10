// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.PIDTargetMeasurement;
import frc.lib.team2930.ShootingSolver;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.visualization.GamepieceVisualization;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ScoreSpeaker extends Command {
  private Drivetrain drive;
  private Shooter shooter;

  private final LoggedTunableNumber rotationKp =
      new LoggedTunableNumber("RotateToSpeaker/rotationKp", 4.9);
  private final LoggedTunableNumber rotationKd =
      new LoggedTunableNumber("RotateToSpeaker/rotationKd", 0.0);

  private final PIDController rotationController;

  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;

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
          new Translation3d(),
          Constants.ShooterConstants.SHOOTING_SPEED,
          Constants.ShooterConstants.SHOOTING_TIME);

  private BooleanSupplier shootGamepiece;

  private boolean shooting;

  private boolean prevShooting;

  private GamepieceVisualization gamepieceVisualizer = new GamepieceVisualization();

  /**
   * Creates a new RotateToSpeaker.
   *
   * @param translationXSupplier controller horizontal translation output
   * @param translationYSupplier controller vertical translation output
   * @param drive drivetrain subsystem
   * @param shooter shooter subsystem
   * @param shootGamepiece when this command should end
   * @return Command to lock rotation in direction of target
   */
  public ScoreSpeaker(
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      Drivetrain drive,
      Shooter shooter,
      BooleanSupplier shootGamepiece) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.drive = drive;
    this.shooter = shooter;
    this.shootGamepiece = shootGamepiece;

    this.rotationController = new PIDController(rotationKp.get(), 0, rotationKd.get());
    addRequirements(drive);
    setName("ScoreSpeaker");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble()),
            DEADBAND);
    Rotation2d linearDirection =
        new Rotation2d(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble());

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    var currentTime = Timer.getFPGATimestamp();
    var poseEstimatorPose = drive.getPoseEstimatorPose();
    var currentRot = drive.getRotation();

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
      targetRotation = result.heading().getRadians() - Math.PI;
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
        (rotationController.calculate(currentRot.getRadians(), targetRotation)
            + targetAngularSpeed);

    rotationalEffort =
        Math.copySign(
            Math.min(Math.abs(rotationalEffort), drive.getMaxAngularSpeedRadPerSec()),
            rotationalEffort);

    var xVel = linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec();
    var yVel = linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec();

    Logger.recordOutput("ScoreSpeaker/targetRotationDegrees", Math.toDegrees(targetRotation));

    Logger.recordOutput(
        "ScoreSpeaker/targetPose",
        new Pose2d(poseEstimatorPose.getTranslation(), Rotation2d.fromRadians(targetRotation)));

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotationalEffort, drive.getRotation()),
        true);

    // TODO: remove most of these once we are happy with the command
    Logger.recordOutput("ScoreSpeaker/RotationalEffort", rotationalEffort);
    Logger.recordOutput(
        "ScoreSpeaker/rotationalErrorDegrees",
        Units.radiansToDegrees(rotationController.getPositionError()));
    Logger.recordOutput("ScoreSpeaker/desiredLinearVelocity", linearVelocity);
    Logger.recordOutput("ScoreSpeaker/feedForward", targetAngularSpeed);
    Logger.recordOutput("ScoreSpeaker/robotRotationDegrees", drive.getRotation().getDegrees());
    Logger.recordOutput("ScoreSpeaker/targetRotationDegrees", targetRotation);
    Logger.recordOutput("ScoreSpeaker/atSetpoint", rotationController.atSetpoint());

    if (rotationKp.hasChanged(hashCode()) || rotationKd.hasChanged(hashCode())) {
      rotationController.setP(rotationKp.get());
      rotationController.setD(rotationKd.get());
    }

    shooter.setPercentOut(Constants.ShooterConstants.SHOOTING_PERCENT_OUT);

    shooter.setPivotPosition(targetPitch);

    rotationController.setTolerance(
        Units.degreesToRadians(
            (Units.feetToMeters(15.0)
                / Math.hypot(
                    Constants.FieldConstants.getSpeakerTranslation().getX()
                        - drive.getPoseEstimatorPose().getX(),
                    Constants.FieldConstants.getSpeakerTranslation().getY()
                        - drive.getPoseEstimatorPose().getY()))));

    shooting =
        shootGamepiece.getAsBoolean()
            && shooter.isPivotIsAtTarget()
            && rotationController.atSetpoint()
            && shooter.isAtTargetRPM();

    Logger.recordOutput("ScoreSpeaker/shooting", shooting);

    if (shooting) {
      shooter.setKickerPercentOut(Constants.ShooterConstants.Kicker.KICKING_PERCENT_OUT);
    } else {
      shooter.setKickerPercentOut(0.0);
      solver.endShooting();
    }

    if (shooting && !prevShooting) {
      solver.startShooting(Timer.getFPGATimestamp());
    }

    prevShooting = shooting;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0, 0, 0), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  boolean prevIsShooting = false;

  boolean isShot = false;

  boolean isShotPrev = false;

  boolean shotStart = false;

  Timer shootingTimer = new Timer();

  Pose2d robotPoseOfShot = new Pose2d();

  public void updateVisualization() {
    boolean isShooting = shooting;

    if (!isShooting) {
      shootingTimer.stop();
      shootingTimer.reset();
    }

    boolean shootingStart = false;

    shootingStart = isShooting && !prevIsShooting;

    if (shootingStart) shootingTimer.start();

    gamepieceVisualizer.updateVisualization(
        drive.getPoseEstimatorPose(),
        drive.getFieldRelativeVelocities().getTranslation(),
        shooter.getPitch(),
        shooter.getRPM(),
        isShooting,
        shootingTimer.get());

    prevIsShooting = isShooting;

    gamepieceVisualizer.logTraj();
  }
}
