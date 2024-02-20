// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterStowAndStopRollers extends Command {
  private Shooter shooter;

  /** Creates a new ShooterDefaultCommand. */
  public ShooterStowAndStopRollers(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    addRequirements(shooter);
    setName("ShooterStowAndStopRollers");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setPivotPosition(ShooterConstants.Pivot.SHOOTER_STOW_PITCH);
    shooter.setLauncherVoltage(0.0);
    shooter.setKickerPercentOut(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
