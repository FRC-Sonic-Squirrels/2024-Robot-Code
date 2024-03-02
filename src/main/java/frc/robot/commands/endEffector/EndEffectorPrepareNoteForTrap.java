// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.endEffector.EndEffector;

public class EndEffectorPrepareNoteForTrap extends Command {
  /** Creates a new EndEffectorPrepareNoteForTrap. */
  EndEffector endEffector;

  Trigger gamepieceNotInIntakeSideTOF;

  public EndEffectorPrepareNoteForTrap(EndEffector endEffector) {
    this.endEffector = endEffector;

    gamepieceNotInIntakeSideTOF =
        new Trigger(() -> !endEffector.shooterSideTOFDetectGamepiece())
            .debounce(1.0, DebounceType.kRising);

    addRequirements(endEffector);
    setName("endEffectorPrepareForTrap");
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (endEffector.shooterSideTOFDetectGamepiece()) {
      endEffector.setPercentOut(-0.05);
    } else {
      endEffector.setPercentOut(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setPercentOut(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gamepieceNotInIntakeSideTOF.getAsBoolean();
  }
}
