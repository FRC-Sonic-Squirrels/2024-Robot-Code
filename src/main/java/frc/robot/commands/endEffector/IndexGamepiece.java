// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.endEffector.EndEffector;

public class IndexGamepiece extends Command {
  private final EndEffector endEffector;

  /** Creates a new advanceGamepiece. */
  public IndexGamepiece(EndEffector endEffector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.endEffector = endEffector;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    endEffector.setPercentOut(Constants.EndEffectorConstants.INDEX_PERCENT_OUT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setPercentOut(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
