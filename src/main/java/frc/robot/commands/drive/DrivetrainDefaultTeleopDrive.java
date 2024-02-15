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
import frc.robot.DrivetrainWrapper;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DrivetrainDefaultTeleopDrive extends Command {
  /** Creates a new DrivetrainDefaultTeleopDrive. */
  private final DrivetrainWrapper drivetrain;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;

  private double DEADBAND = 0.1;

  private BooleanSupplier resetRotationOffset;

  private Rotation2d rotationOffset = new Rotation2d();

  public DrivetrainDefaultTeleopDrive(
      DrivetrainWrapper drivetrain,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier resetRotationOffset) {

    this.drivetrain = drivetrain;

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    this.resetRotationOffset = resetRotationOffset;

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
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    var correctedLinearVelocity = AllianceFlipUtil.flipVelocitiesForAlliance(linearVelocity);

    if (resetRotationOffset.getAsBoolean()) {
      rotationOffset = drivetrain.getPoseEstimatorPose().getRotation().unaryMinus();
    }

    Logger.recordOutput("Commands/TeleopDrive/XSupplier", xSupplier.getAsDouble());
    Logger.recordOutput("Commands/TeleopDrive/YSupplier", ySupplier.getAsDouble());
    Logger.recordOutput("Commands/TeleopDrive/OmegaSupplier", omegaSupplier.getAsDouble());

    Logger.recordOutput("Commands/TeleopDrive/LinearMagnitude", linearMagnitude);
    Logger.recordOutput("Commands/TeleopDrive/LinearDirection", linearDirection);

    Logger.recordOutput("Commands/TeleopDrive/LinearVelocity", linearVelocity);

    // Convert to field relative speeds & send command
    ChassisSpeeds chassisSpeeds =
        new ChassisSpeeds(
            correctedLinearVelocity.getX() * drivetrain.getMaxLinearSpeedMetersPerSec(),
            correctedLinearVelocity.getY() * drivetrain.getMaxLinearSpeedMetersPerSec(),
            omega * drivetrain.getMaxAngularSpeedRadPerSec());

    drivetrain.setVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            chassisSpeeds, drivetrain.getRotation().plus(rotationOffset)));
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
