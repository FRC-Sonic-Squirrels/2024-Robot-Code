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
  private static final String ROOT_TABLE = "Commands/TeleopDrive";

  private static final LoggerGroup logGroup = new LoggerGroup(ROOT_TABLE);
  private static final LoggerEntry log_XSupplier = logGroup.build("XSupplier");
  private static final LoggerEntry log_YSupplier = logGroup.build("YSupplier");
  private static final LoggerEntry log_OmegaSupplier = logGroup.build("OmegaSupplier");
  private static final LoggerEntry log_LinearMagnitude = logGroup.build("LinearMagnitude");
  private static final LoggerEntry log_LinearDirection = logGroup.build("LinearDirection");
  private static final LoggerEntry log_LinearVelocity = logGroup.build("LinearVelocity");

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
    // Use addRequirements() here to declare subsystem dependencies.
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
