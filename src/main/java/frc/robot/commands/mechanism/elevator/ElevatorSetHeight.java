// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanism.elevator;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.Supplier;

public class ElevatorSetHeight extends Command {
  private static final String ROOT_TABLE = "ElevatorSetHeight";

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Decimal log_targetHeight = logGroup.buildDecimal("targetHeight");

  /** Creates a new ElevatorSetHeight. */
  private final Elevator elevator;

  private final Supplier<Measure<Distance>> heightSupplier;

  public ElevatorSetHeight(Elevator elevator, Supplier<Measure<Distance>> heightSupplier) {
    this.elevator = elevator;
    this.heightSupplier = heightSupplier;
    addRequirements(elevator);
    setName("ElevatorSetHeight");
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
    log_targetHeight.info(heightSupplier.get().in(Units.Inches));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
