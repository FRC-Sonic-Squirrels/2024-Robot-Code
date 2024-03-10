// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class LoadGamepieceToShooter extends Command {
  /** Creates a new LoadGamepieceToShooter. */
  private final Shooter shooter;

  private final EndEffector endEffector;
  private final Intake intake;

  public LoadGamepieceToShooter(Shooter shooter, EndEffector endEffector, Intake intake) {
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, endEffector, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setPivotPosition(Constants.ShooterConstants.Pivot.MIN_ANGLE_RAD);

    double percentOut =
        shooter.noteInShooter()
                || !shooter.isPivotIsAtTarget(Constants.ShooterConstants.Pivot.MIN_ANGLE_RAD)
            ? 0.0
            : 0.1;
    shooter.setKickerPercentOut(percentOut);
    endEffector.setPercentOut(percentOut);
    intake.setPercentOut(percentOut);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setKickerPercentOut(0.0);
    endEffector.setPercentOut(0.0);
    intake.setPercentOut(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.noteInShooter();
  }
}
