package frc.robot.commands.elevator;

import frc.lib.team2930.commands.ConsumeSuppliedValue;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.Supplier;

public class ElevatorManualControl extends ConsumeSuppliedValue {

  public ElevatorManualControl(Elevator elevator, Supplier<Double> volts) {
    super(elevator, volts, elevator::setVoltage);
  }
}
