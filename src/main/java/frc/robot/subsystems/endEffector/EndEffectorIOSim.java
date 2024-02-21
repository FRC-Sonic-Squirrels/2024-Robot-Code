package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;

public class EndEffectorIOSim implements EndEffectorIO {

  private DCMotorSim motor =
      new DCMotorSim(
          DCMotor.getFalcon500(1),
          Constants.EndEffectorConstants.GEARING,
          Constants.EndEffectorConstants.MOI);

  private static final TunableNumberGroup group = new TunableNumberGroup("sim_EndEffector");

  private LoggedTunableNumber shooterTOFInches = group.build("shooterTOFInches", 18);
  private LoggedTunableNumber intakeTOFInches = group.build("intakeTOFInches", 18);

  private double voltage = 0.0;

  public EndEffectorIOSim() {}

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    motor.update(0.02);
    motor.setInputVoltage(voltage);

    inputs.intakeSideTOFDistanceInches = intakeTOFInches.get();
    inputs.shooterSideTOFDistanceInches = shooterTOFInches.get();
  }

  @Override
  public void setVoltage(double volts) {
    voltage = volts;
  }
}
