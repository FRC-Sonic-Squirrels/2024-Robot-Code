// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.shooter.Shooter;

public class HomeShooter extends Command {
  private static final TunableNumberGroup group = new TunableNumberGroup("HomeShooter");

  private static final LoggedTunableNumber homingVoltage = group.build("homingVoltage", -0.1);
  private static final LoggedTunableNumber homingVelocityMaxToResetShooter =
      group.build("homingVelocityMaxToResetShooter", 0.02);

  private final Shooter shooter;
  private boolean shooterReset = false;

  /** Creates a new HomeShooter. */
  public HomeShooter(Shooter shooter) {
    this.shooter = shooter;

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
      shooter.setPivotVoltage(homingVoltage.get());

      if (Math.abs(shooter.getPivotVoltage()) >= Math.abs(homingVoltage.get()) / 2.0
          && shooter.getPivotVelocity() <= homingVelocityMaxToResetShooter.get()) {
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
