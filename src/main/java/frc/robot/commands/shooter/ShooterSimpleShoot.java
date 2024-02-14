// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSimpleShoot extends Command {
  private final Shooter shooter;
  private final EndEffector endEffector;
  private final DoubleSupplier rpmSupplier;
  private final Supplier<Rotation2d> pitchSupplier;
  private final DoubleSupplier kickerPercentOutSupplier;
  private final DoubleSupplier endEffectorPercentOutSupplier;

  public ShooterSimpleShoot(
      Shooter shooter,
      EndEffector endEffector,
      DoubleSupplier rpmSupplier,
      Supplier<Rotation2d> pitchSupplier,
      DoubleSupplier kickerPercentOutSupplier,
      DoubleSupplier endEffectorPercentOutSupplier) {

    this.shooter = shooter;
    this.endEffector = endEffector;
    this.rpmSupplier = rpmSupplier;
    this.pitchSupplier = pitchSupplier;
    this.kickerPercentOutSupplier = kickerPercentOutSupplier;
    this.endEffectorPercentOutSupplier = endEffectorPercentOutSupplier;

    addRequirements(shooter, endEffector);
  }

  public ShooterSimpleShoot(
      Shooter shooter,
      EndEffector endEffector,
      double rpm,
      Rotation2d pitch,
      double kickerPercentOut,
      double endEffectorPercentOut) {
    this(
        shooter,
        endEffector,
        () -> rpm,
        () -> pitch,
        () -> kickerPercentOut,
        () -> endEffectorPercentOut);
  }

  public ShooterSimpleShoot(
      Shooter shooter, EndEffector endEffector, Supplier<Rotation2d> pitchSupplier) {
    this(
        shooter,
        endEffector,
        () -> Constants.ShooterConstants.SHOOTING_RPM,
        pitchSupplier,
        () -> Constants.ShooterConstants.Kicker.KICKING_PERCENT_OUT,
        () -> Constants.EndEffectorConstants.SHOOTING_PERCENT_OUT);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setLauncherRPM(rpmSupplier.getAsDouble());
    shooter.setPivotPosition(pitchSupplier.get());

    var validShot = shooter.isPivotIsAtTarget() && shooter.isAtTargetRPM();

    if (validShot) {
      shooter.setKickerPercentOut(kickerPercentOutSupplier.getAsDouble());
      endEffector.setPercentOut(endEffectorPercentOutSupplier.getAsDouble());
    } else {
      shooter.setKickerPercentOut(0.0);
      endEffector.setPercentOut(0.0);
    }

    Logger.recordOutput("ShooterSimpleShot/validShot", validShot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setKickerPercentOut(0.0);
    endEffector.setPercentOut(0.0);
    shooter.setLauncherRPM(0.0);

    shooter.setPivotPosition(Constants.ShooterConstants.Pivot.SHOOTER_STOW_PITCH);

    Logger.recordOutput("ShooterSimpleShot/validShot", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
