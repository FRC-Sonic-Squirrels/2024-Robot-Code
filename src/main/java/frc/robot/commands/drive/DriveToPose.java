// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.GeomUtil;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  private final DrivetrainWrapper drive;
  private final boolean slowMode;
  private final Supplier<Pose2d> poseSupplier;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private Translation2d lastSetpointTranslation;

  private static final TunableNumberGroup group = new TunableNumberGroup("DriveToPose");

  private static final LoggedTunableNumber driveKp = group.build("DriveKp");
  private static final LoggedTunableNumber driveKd = group.build("DriveKd");
  private static final LoggedTunableNumber thetaKp = group.build("ThetaKp");
  private static final LoggedTunableNumber thetaKd = group.build("ThetaKd");
  private static final LoggedTunableNumber driveMaxVelocity = group.build("DriveMaxVelocity");
  private static final LoggedTunableNumber driveMaxVelocitySlow =
      group.build("DriveMaxVelocitySlow");
  private static final LoggedTunableNumber driveMaxAcceleration =
      group.build("DriveMaxAcceleration");
  private static final LoggedTunableNumber thetaMaxVelocity = group.build("ThetaMaxVelocity");
  private static final LoggedTunableNumber thetaMaxVelocitySlow =
      group.build("ThetaMaxVelocitySlow");
  private static final LoggedTunableNumber thetaMaxAcceleration =
      group.build("ThetaMaxAcceleration");
  private static final LoggedTunableNumber driveTolerance = group.build("DriveTolerance");
  private static final LoggedTunableNumber driveToleranceSlow = group.build("DriveToleranceSlow");
  private static final LoggedTunableNumber thetaTolerance = group.build("ThetaTolerance");
  private static final LoggedTunableNumber thetaToleranceSlow = group.build("ThetaToleranceSlow");
  private static final LoggedTunableNumber ffMinRadius = group.build("FFMinRadius");
  private static final LoggedTunableNumber ffMaxRadius = group.build("FFMinRadius");

  static {
    driveKp.initDefault(2.0);
    driveKd.initDefault(0.0);
    thetaKp.initDefault(5.0);
    thetaKd.initDefault(0.0);
    driveMaxVelocity.initDefault(Units.Inches.of(150.0).in(Units.Meters));
    driveMaxVelocitySlow.initDefault(Units.Inches.of(50.0).in(Units.Meters));
    driveMaxAcceleration.initDefault(Units.Inches.of(95.0).in(Units.Meters));
    thetaMaxVelocity.initDefault(Math.toRadians(360.0));
    thetaMaxVelocitySlow.initDefault(Math.toRadians(90.0));
    thetaMaxAcceleration.initDefault(Math.toRadians(720.0));
    driveTolerance.initDefault(0.01);
    driveToleranceSlow.initDefault(0.06);
    thetaTolerance.initDefault(Math.toRadians(1.0));
    thetaToleranceSlow.initDefault(Math.toRadians(3.0));
    ffMinRadius.initDefault(0.2);
    ffMaxRadius.initDefault(0.8);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(DrivetrainWrapper drive, Pose2d pose) {
    this(drive, false, pose);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(DrivetrainWrapper drive, boolean slowMode, Pose2d pose) {
    this(drive, slowMode, () -> pose);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(DrivetrainWrapper drive, Supplier<Pose2d> poseSupplier) {
    this(drive, false, poseSupplier);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(DrivetrainWrapper drive, boolean slowMode, Supplier<Pose2d> poseSupplier) {
    this.drive = drive;
    this.slowMode = slowMode;
    this.poseSupplier = poseSupplier;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    drive.resetVelocityOverride();
    // Reset all controllers
    var pose = poseSupplier.get();
    var currentPose = drive.getPoseEstimatorPose();
    driveController.reset(
        currentPose.getTranslation().getDistance(pose.getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(
                    drive.getFieldRelativeVelocities().getX(),
                    drive.getFieldRelativeVelocities().getY())
                .rotateBy(
                    pose.getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(
        currentPose.getRotation().getRadians(),
        drive.getFieldRelativeVelocities().getRotation().getRadians());
    lastSetpointTranslation = currentPose.getTranslation();

    // Update from tunable numbers
    driveController.setP(driveKp.get());
    driveController.setD(driveKd.get());
    driveController.setConstraints(
        new TrapezoidProfile.Constraints(
            slowMode ? driveMaxVelocitySlow.get() : driveMaxVelocity.get(),
            driveMaxAcceleration.get()));
    driveController.setTolerance(slowMode ? driveToleranceSlow.get() : driveTolerance.get());
    thetaController.setP(thetaKp.get());
    thetaController.setD(thetaKd.get());
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(
            slowMode ? thetaMaxVelocitySlow.get() : thetaMaxVelocity.get(),
            thetaMaxAcceleration.get()));
    thetaController.setTolerance(slowMode ? thetaToleranceSlow.get() : thetaTolerance.get());
  }

  @Override
  public void execute() {
    // Get current and target pose
    var currentPose = drive.getPoseEstimatorPose();
    var targetPose = poseSupplier.get();

    // Calculate drive speed
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    // Command speeds
    var driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
            .getTranslation();
    drive.setVelocityOverride(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    // Log data
    Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "Odometry/DriveToPoseSetpoint",
        new Pose2d(
            lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
    Logger.recordOutput("Odometry/DriveToPoseGoal", targetPose);
    Logger.recordOutput("DriveToPose/isScheduled", isScheduled());
    Logger.recordOutput("DriveToPose/DriveControllerIsAtGoal", driveController.atGoal());
    Logger.recordOutput("DriveToPose/ThetaControllerIsAtGoal", thetaController.atGoal());
  }

  @Override
  public void end(boolean interrupted) {
    drive.resetVelocityOverride();
    Logger.recordOutput("Odometry/DriveToPoseSetpoint", new double[] {});
    Logger.recordOutput("Odometry/DriveToPoseGoal", new double[] {});
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return
    // isScheduled() &&
    driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return
    // isScheduled() &&
    Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }
}
