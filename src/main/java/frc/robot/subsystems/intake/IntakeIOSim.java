package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class IntakeIOSim implements IntakeIO {

  private DCMotorSim motor =
      new DCMotorSim(
          DCMotor.getFalcon500(1),
          Constants.IntakeConstants.GEARING,
          Constants.IntakeConstants.MOI);

  private double voltage = 0.0;

  private LoggedDashboardBoolean beamBreak = new LoggedDashboardBoolean("Intake/BeamBreak", false);

  public IntakeIOSim() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    motor.setInputVoltage(voltage);
    inputs.currentAmps = motor.getCurrentDrawAmps();
    inputs.RPM = motor.getAngularVelocityRPM();
    inputs.beamBreak = beamBreak.get();
  }

  @Override
  public void setVoltage(double volts) {
    volts = voltage;
  }
}
