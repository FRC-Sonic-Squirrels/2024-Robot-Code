package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class EndEffectorIOSim implements EndEffectorIO {

  private DCMotorSim motor =
      new DCMotorSim(
          DCMotor.getFalcon500(1),
          Constants.EndEffectorConstants.GEARING,
          Constants.EndEffectorConstants.MOI);

  private LoggedDashboardBoolean beamBreak =
      new LoggedDashboardBoolean("EndEffector/beamBreak", false);

  private LoggedTunableNumber shooterTOFInches =
      new LoggedTunableNumber("sim_EndEffector/shooterTOFInches", 0);

  private LoggedTunableNumber intakeTOFInches =
      new LoggedTunableNumber("sim_EndEffector/intakeTOFInches", 0);

  private double voltage = 0.0;

  public EndEffectorIOSim() {}

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    motor.update(0.02);
    motor.setInputVoltage(voltage);
    inputs.RPM = motor.getAngularVelocityRPM();
  }

  @Override
  public void setVoltage(double volts) {
    voltage = volts;
  }
}
