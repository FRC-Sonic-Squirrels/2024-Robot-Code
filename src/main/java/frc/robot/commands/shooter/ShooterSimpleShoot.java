// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class ShooterSimpleShoot extends Command {
  /** Creates a new ShooterSimpleShoot. */
  public ShooterSimpleShoot(
      Shooter shooter,
      EndEffector endEffector,
      DoubleSupplier rpmSupplier,
      DoubleSupplier pitchSupplier,
      DoubleSupplier kickerPercentOutSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ShooterSimpleShoot(
      Shooter shooter, EndEffector endEffector, double rpm, double pitch, double kickerPercentOut) {
    this(shooter, endEffector, () -> rpm, () -> pitch, () -> kickerPercentOut);
  }

  public ShooterSimpleShoot(
      Shooter shooter, EndEffector endEffector, DoubleSupplier pitchSupplier) {
    this(
        shooter,
        endEffector,
        () -> Constants.ShooterConstants.SHOOTING_RPM,
        pitchSupplier,
        () -> Constants.ShooterConstants.Kicker.KICKING_PERCENT_OUT);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
