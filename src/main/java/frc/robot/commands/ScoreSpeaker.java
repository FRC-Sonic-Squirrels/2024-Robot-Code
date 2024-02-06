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
  private LinearFilter pidLatencyfilter = LinearFilter.movingAverage(5);

  private ArrayList<PIDTargetMeasurement> targetMeasurements =
      new ArrayList<PIDTargetMeasurement>();

  private double pidLatency = 0.0;

  private ShootingSolver solver =
      new ShootingSolver(
          Constants.FieldConstants.getSpeakerTranslation3D(), new Translation3d(), 5800.0);

  private BooleanSupplier shootGamepiece;

  private boolean shooting;

  /**
   * Creates a new RotateToSpeaker.
   *
   * @param translationXSupplier controller horizontal translation output
   * @param translationYSupplier controller vertical translation output
   * @param targetPose pose to target
   * @param drive drivetrain subsystem
   * @param endCondition when this command should end
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
    setName("RotateToSpeaker");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(Units.degreesToRadians(5));
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

    var currentRot = drive.getRotation();

    var result =
        solver.computeRobotYaw(
            pidLatency,
            drive.getPoseEstimatorPose().getTranslation(),
            drive.getFieldRelativeVelocities().getTranslation(),
            drive.getFieldRelativeAccelerations());

    Rotation2d targetRotation;
    Rotation2d targetAngularSpeed;

    if (result == null) {
      targetRotation = drive.getPoseEstimatorPose().getRotation();
      targetAngularSpeed = new Rotation2d(0.0);
    } else {
      targetRotation = result.heading();
      targetAngularSpeed = result.angularVel();
    }

    targetMeasurements.add(
        new PIDTargetMeasurement(
            Timer.getFPGATimestamp(),
            targetRotation.getRadians(),
            drive.getRotation().getRadians() <= targetRotation.getRadians()));
    for (int index = 0; index < targetMeasurements.size(); index++) {
      PIDTargetMeasurement measurement = targetMeasurements.get(index);
      if ((measurement.upDirection
              && drive.getRotation().getRadians() >= measurement.targetRot.getRadians())
          || (!measurement.upDirection
              && drive.getRotation().getRadians() <= measurement.targetRot.getRadians())) {
        pidLatency = pidLatencyfilter.calculate(Timer.getFPGATimestamp() - measurement.timestamp);
        targetMeasurements.remove(index);
      } else if (Timer.getFPGATimestamp() - measurement.timestamp >= 3.0) {
        targetMeasurements.remove(index);
      }
    }

    Logger.recordOutput("RotateToSpeaker/PIDLatency", pidLatency);

    var rotationalEffort =
        (rotationController.calculate(currentRot.getRadians(), targetRotation.getRadians())
            - targetAngularSpeed.getRadians());

    rotationalEffort =
        Math.copySign(
            Math.min(Math.abs(rotationalEffort), drive.getMaxAngularSpeedRadPerSec()),
            rotationalEffort);

    var xVel = linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec();
    var yVel = linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec();

    Rotation2d optimalAngle =
        solver
            .computeRobotYaw(
                0.0,
                drive.getPoseEstimatorPose().getTranslation(),
                drive.getFieldRelativeVelocities().getTranslation(),
                drive.getFieldRelativeAccelerations())
            .heading();

    Logger.recordOutput("RotateToSpeaker/optimalRotationDegrees", optimalAngle.getDegrees());

    Logger.recordOutput(
        "RotateToSpeaker/optimalPose",
        new Pose2d(drive.getPoseEstimatorPose().getTranslation(), optimalAngle));

    drive.runVelocityPrioritizeRotation(
        ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotationalEffort, drive.getRotation()));

    // TODO: remove most of these once we are happy with the command
    Logger.recordOutput("RotateToSpeaker/RotationalEffort", rotationalEffort);
    Logger.recordOutput(
        "RotateToSpeaker/rotationalErrorDegrees",
        Units.radiansToDegrees(rotationController.getPositionError()));
    Logger.recordOutput("RotateToSpeaker/desiredLinearVelocity", linearVelocity);
    Logger.recordOutput("RotateToSpeaker/feedForward", -targetAngularSpeed.getRadians());
    Logger.recordOutput("RotateToSpeaker/robotRotationDegrees", drive.getRotation().getDegrees());
    Logger.recordOutput("RotateToSpeaker/targetRotationDegrees", targetRotation.getDegrees());
    Logger.recordOutput("RotateToSpeaker/atSetpoint", rotationController.atSetpoint());

    if (rotationKp.hasChanged(hashCode()) || rotationKd.hasChanged(hashCode())) {
      rotationController.setP(rotationKp.get());
      rotationController.setD(rotationKd.get());
    }

    shooter.setPercentOut(Constants.ShooterConstants.SHOOTING_RPM);

    // shooter.setPivotPosition();

    shooting =
        shootGamepiece.getAsBoolean()
            && shooter.pivotIsAtTarget()
            && rotationController.atSetpoint();

    Logger.recordOutput("ShooterShootMode/shooting", shooting);

    if (shooting) {
      shooter.setKickerPercentOut(Constants.ShooterConstants.Kicker.KICKING_PERCENT_OUT);
    } else {
      shooter.setKickerPercentOut(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isShooting() {
    return shooting;
  }
}
