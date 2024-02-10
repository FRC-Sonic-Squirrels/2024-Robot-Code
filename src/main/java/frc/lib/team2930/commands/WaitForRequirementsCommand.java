// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team2930.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * When scheduling commands that have conflicting requirements, only one would win in the stock WPI
 * code. This command keeps monitoring the in-use requirements and only tries to schedule the target
 * command when the coast is clear.
 */
public class WaitForRequirementsCommand extends Command {
  private Command command;

  public WaitForRequirementsCommand(Command command) {
    this.command = command;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (command != null) {
      var scheduler = CommandScheduler.getInstance();

      // Keep checking if any of the requirements are in use.
      for (var requirement : command.getRequirements()) {
        if (scheduler.requiring(requirement) != null) return;
      }

      // If not, we can schedule the actual command.
      command.schedule();
      command = null;
    }
  }

  @Override
  public boolean isFinished() {
    // Once we schedule the command, we can end.
    return command == null;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
