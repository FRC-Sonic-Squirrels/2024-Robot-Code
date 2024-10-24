package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team2930.IsaacSimDispatcher;
import frc.robot.Constants;

public class ArmIOIsaacSim implements ArmIO {

  private IsaacSimDispatcher dispatcher;

  public ArmIOIsaacSim(IsaacSimDispatcher dispatcher) {
    this.dispatcher = dispatcher;
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.armPosition =
        Rotation2d.fromRadians(dispatcher.recieveMotorPos(Constants.CanIDs.ARM_CAN_ID)).minus(Rotation2d.fromDegrees(90));
    inputs.armAngleDegrees = inputs.armPosition.getDegrees();
    inputs.armAppliedVolts = 0;
    inputs.armCurrentAmps = 0;
    inputs.armTempCelsius = 0;
    inputs.armVelocity = dispatcher.recieveMotorVel(Constants.CanIDs.ARM_CAN_ID);
  }

  @Override
  public void setClosedLoopPosition(Rotation2d angle) {
    dispatcher.sendMotorInfo(Constants.CanIDs.ARM_CAN_ID, angle.getRadians());
  }

  @Override
  public void resetSensorPosition(Rotation2d angle) {}
}
