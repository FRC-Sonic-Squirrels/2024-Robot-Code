// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorSetHeight extends Command {
  /** Creates a new ElevatorSetHeight. */
  Elevator elevator;
  DoubleSupplier heightSupplier;
  double height;
  public ElevatorSetHeight(Elevator elevator, DoubleSupplier heightSupplier) {
    this.heightSupplier = heightSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  public ElevatorSetHeight(Elevator elevator, double height) {
    this(elevator, () -> height);
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setHeight(height);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
