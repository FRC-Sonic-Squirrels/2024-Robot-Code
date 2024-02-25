// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Shimmy extends Command {
  Timer time = new Timer();
  private Intake intake;
  private EndEffector endEffector;
  private Shooter shooter;

  /** Creates a new Shimmy. */
  public Shimmy(Intake intake, EndEffector endEffector, Shooter shooter) {
    this.intake = intake;
    this.endEffector = endEffector;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double percentOut = Math.sin(time.get() / (2 * Math.PI)) * 0.7;

    intake.setPercentOut(percentOut);
    shooter.setKickerPercentOut(percentOut);
    endEffector.setPercentOut(percentOut);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPercentOut(0.0);
    shooter.setKickerPercentOut(0.0);
    endEffector.setPercentOut(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
