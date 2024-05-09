// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team2930.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class ConsumeSuppliedValue extends Command {
  /** Creates a new ConsumeTunableValue. */
  private final Consumer<Double> consumer;

  private final Supplier<Double> supplier;

  /**
   * Command to periodically accept supplied value
   *
   * @param subsystem
   * @param supplier
   * @param consumer
   */
  public ConsumeSuppliedValue(
      Subsystem subsystem, Supplier<Double> supplier, Consumer<Double> consumer) {
    this.consumer = consumer;
    this.supplier = supplier;

    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    consumer.accept(supplier.get());
  }
}
