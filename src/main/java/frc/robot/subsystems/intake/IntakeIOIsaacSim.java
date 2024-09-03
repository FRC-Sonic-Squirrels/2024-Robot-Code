package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.lib.team2930.IsaacSimDispatcher;
import frc.robot.Constants;

public class IntakeIOIsaacSim implements IntakeIO {

  private IsaacSimDispatcher dispatcher;

  public IntakeIOIsaacSim(IsaacSimDispatcher dispatcher) {
    this.dispatcher = dispatcher;
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.currentAmps = 0;
    inputs.tempCelsius = 0;
    inputs.appliedVolts = 0;
    inputs.velocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(
            dispatcher.recieveMotorVel(Constants.CanIDs.INTAKE_CAN_ID));
  }

  @Override
  public void setVoltage(double volts) {
    dispatcher.sendMotorInfo(Constants.CanIDs.INTAKE_CAN_ID, 0);
  }

  @Override
  public void setVelocity(double revPerMin) {
    dispatcher.sendMotorInfo(
        Constants.CanIDs.INTAKE_CAN_ID, Units.rotationsPerMinuteToRadiansPerSecond(revPerMin));
  }
}
