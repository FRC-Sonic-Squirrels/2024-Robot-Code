// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ElevatorSetHeight extends Command {
  /** Creates a new ElevatorSetHeight. */
  Elevator elevator;

  Supplier<Measure<Distance>> heightSupplier;

  public ElevatorSetHeight(Elevator elevator, Supplier<Measure<Distance>> heightSupplier) {
    this.elevator = elevator;
    this.heightSupplier = heightSupplier;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  public ElevatorSetHeight(Elevator elevator, Measure<Distance> height) {
    this(elevator, () -> height);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setHeight(heightSupplier.get());
    Logger.recordOutput("ElevatorSetHeight/targetHeight", heightSupplier.get().in(Units.Inches));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isAtTarget();
  }
}
