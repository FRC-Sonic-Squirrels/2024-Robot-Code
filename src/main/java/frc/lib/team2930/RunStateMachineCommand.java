// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team2930;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

public class RunStateMachineCommand extends Command {
  private final Supplier<StateMachine> supplier;
  private StateMachine stateMachine;

  public RunStateMachineCommand(
      Supplier<StateMachine> stateMachineSupplier, Subsystem... subsystems) {
    this.supplier = stateMachineSupplier;

    addRequirements(subsystems);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateMachine = supplier.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    stateMachine.advance();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateMachine = null;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stateMachine == null || !stateMachine.isRunning();
  }
}
