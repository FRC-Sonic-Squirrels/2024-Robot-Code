// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;

public class HomeShooter extends Command {
  private Shooter shooter;
  private boolean homeShooter = false;
  private boolean shooterReset = false;
  private final Rotation2d safePivotAngle =
      Constants.ShooterConstants.Pivot.HOME_POSITION.plus(Rotation2d.fromDegrees(2.0));

  /** Creates a new HomeMechanism. */
  public HomeShooter(Shooter shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    setName("HomeShooter");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!shooterReset) {
      if (shooter.isPivotIsAtTarget(safePivotAngle)) {
        homeShooter = true;
      }

      if (homeShooter) {
        shooter.setPivotVoltage(-0.1);
      } else {
        shooter.setPivotPosition(safePivotAngle);
      }

      if (shooter.getPivotVoltage() <= -0.05 && shooter.getPivotVelocity() >= -0.02) {
        shooter.pivotResetHomePosition();
        shooterReset = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterReset;
  }
}
