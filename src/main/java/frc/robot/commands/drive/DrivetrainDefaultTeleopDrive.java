// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.DoubleSupplier;

public class DrivetrainDefaultTeleopDrive extends Command {
  private static final String ROOT_TABLE = "TeleopDrive";

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Decimal log_XSupplier = logGroup.buildDecimal("XSupplier");
  private static final LoggerEntry.Decimal log_YSupplier = logGroup.buildDecimal("YSupplier");
  private static final LoggerEntry.Decimal log_OmegaSupplier =
      logGroup.buildDecimal("OmegaSupplier");
  private static final LoggerEntry.Decimal log_LinearMagnitude =
      logGroup.buildDecimal("LinearMagnitude");
  private static final LoggerEntry.Decimal log_LinearDirection =
      logGroup.buildDecimal("LinearDirection");
  private static final LoggerEntry.Struct<Translation2d> log_LinearVelocity =
      logGroup.buildStruct(Translation2d.class, "LinearVelocity");

  /** Creates a new DrivetrainDefaultTeleopDrive. */
  private final DrivetrainWrapper drivetrain;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;

  private double DEADBAND = 0.1;

  public DrivetrainDefaultTeleopDrive(
      DrivetrainWrapper drivetrain,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    this.drivetrain = drivetrain;

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;

    addRequirements(drivetrain.getRequirements());
    setName("DrivetrainDefaultTeleopDrive");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega = Math.copySign(omega * omega, omega);

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(Constants.zeroTranslation2d, linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, Constants.zeroRotation2d))
            .getTranslation();

    var correctedLinearVelocity = AllianceFlipUtil.flipVelocitiesForAlliance(linearVelocity);

    log_XSupplier.info(xSupplier.getAsDouble());
    log_YSupplier.info(ySupplier.getAsDouble());
    log_OmegaSupplier.info(omegaSupplier.getAsDouble());

    log_LinearMagnitude.info(linearMagnitude);
    log_LinearDirection.info(linearDirection);
    log_LinearVelocity.info(linearVelocity);

    // Convert to field relative speeds & send command
    ChassisSpeeds chassisSpeeds =
        new ChassisSpeeds(
            correctedLinearVelocity.getX() * drivetrain.getMaxLinearSpeedMetersPerSec(),
            correctedLinearVelocity.getY() * drivetrain.getMaxLinearSpeedMetersPerSec(),
            omega * drivetrain.getMaxAngularSpeedRadPerSec());

    drivetrain.setVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, drivetrain.getRotationGyroOnly()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
