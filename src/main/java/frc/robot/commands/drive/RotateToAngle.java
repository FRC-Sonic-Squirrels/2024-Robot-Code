// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RotateToAngle extends Command {
  private static final String ROOT_TABLE = "RotateToAngle";

  private static final TunableNumberGroup groupTunable = new TunableNumberGroup(ROOT_TABLE);
  private static final LoggedTunableNumber tunableKP = groupTunable.build("kP", 4.9);
  private static final LoggedTunableNumber cruiseVelocity =
      groupTunable.build("cruiseVelocity", 2 * Math.PI);
  private static final LoggedTunableNumber maxAcceleration =
      groupTunable.build("maxAcceleration", 4 * Math.PI);
  private static final LoggedTunableNumber toleranceDegrees =
      groupTunable.build("toleranceDegrees", 1.5);

  private final DrivetrainWrapper wrapper;
  private final Supplier<Rotation2d> angle;
  private final Supplier<Pose2d> currentPose;
  private final ProfiledPIDController rotationalPID =
      new ProfiledPIDController(0, 0, 0, new Constraints(0.0, 0.0));

  /** Creates a new RotateToAngle. */
  public RotateToAngle(
      DrivetrainWrapper wrapper, Supplier<Rotation2d> angle, Supplier<Pose2d> currentPose) {
    this.wrapper = wrapper;
    this.angle = angle;
    this.currentPose = currentPose;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationalPID.reset(
        currentPose.get().getRotation().getRadians(),
        wrapper.getFieldRelativeVelocities().getRotation().getRadians());
    rotationalPID.setP(tunableKP.get());
    rotationalPID.setConstraints(new Constraints(cruiseVelocity.get(), maxAcceleration.get()));
    rotationalPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d target = angle.get();
    double rotVel =
        rotationalPID.calculate(currentPose.get().getRotation().getRadians(), target.getRadians());
    wrapper.setRotationOverride(rotVel);

    Logger.recordOutput(ROOT_TABLE + "/rotVel", rotVel);
    Logger.recordOutput(ROOT_TABLE + "/targetRotation", target.getDegrees());
    Logger.recordOutput(
        ROOT_TABLE + "/currentRotation", currentPose.get().getRotation().getDegrees());
    Logger.recordOutput(
        ROOT_TABLE + "/positionErrorDeg", Math.toDegrees(rotationalPID.getPositionError()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrapper.resetRotationOverride();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean withinTolerance() {
    return Math.abs(Math.toDegrees(rotationalPID.getPositionError())) <= toleranceDegrees.get();
  }
}
