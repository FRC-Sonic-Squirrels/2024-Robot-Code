package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.util.Units;
import frc.lib.team2930.IsaacSimDispatcher;
import frc.robot.Constants;

public class EndEffectorIOIsaacSim implements EndEffectorIO {

  private IsaacSimDispatcher dispatcher;

  public EndEffectorIOIsaacSim(IsaacSimDispatcher dispatcher) {
    this.dispatcher = dispatcher;
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.velocityRPM = 0;
    inputs.currentAmps = 0;
    inputs.appliedVolts = 0;
    inputs.tempCelsius = 0;
    inputs.intakeSideTOFDistanceInches =0;
    inputs.shooterSideTOFDistanceInches =0;
  }

  @Override
  public void setVoltage(double volts) {
    dispatcher.sendMotorInfo(Constants.CanIDs.END_EFFECTOR_CAN_ID, 0);
  }

  @Override
  public void setVelocity(double revPerMin) {
    dispatcher.sendMotorInfo(Constants.CanIDs.END_EFFECTOR_CAN_ID, Units.rotationsPerMinuteToRadiansPerSecond(revPerMin));
  }
}
