package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {

  private DCMotorSim motor =
      new DCMotorSim(
          DCMotor.getFalcon500(1),
          Constants.IntakeConstants.GEARING,
          Constants.IntakeConstants.MOI);

  private double voltage = 0.0;

  public IntakeIOSim() {}

  @Override
  public void updateInputs(Inputs inputs) {
    motor.update(Constants.kDefaultPeriod);
    motor.setInputVoltage(voltage);
    inputs.currentAmps = motor.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    volts = voltage;
  }
}
