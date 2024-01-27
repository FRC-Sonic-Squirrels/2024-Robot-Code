package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ElevatorIOSim implements ElevatorIO {
  public ElevatorIOSim() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {}

  @Override
  public void setVoltage(double volts) {}

  @Override
  public void setHeight(double heightInches) {}

  @Override
  public void setPIDConstraints(double kP, double kD, double kG, Constraints constraints) {}
}
