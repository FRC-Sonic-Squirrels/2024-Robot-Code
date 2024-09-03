package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.lib.team2930.IsaacSimDispatcher;
import frc.robot.Constants;

public class ElevatorIOIsaacSim implements ElevatorIO {

  private IsaacSimDispatcher dispatcher;

  public ElevatorIOIsaacSim(IsaacSimDispatcher dispatcher) {
    this.dispatcher = dispatcher;
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.heightInches =
        Units.Meters.of(dispatcher.recieveMotorPos(Constants.CanIDs.ELEVATOR_CAN_ID))
            .in(Units.Inches);
    inputs.velocityInchesPerSecond =
        Units.MetersPerSecond.of(dispatcher.recieveMotorVel(Constants.CanIDs.ELEVATOR_CAN_ID))
            .in(Units.InchesPerSecond);
    inputs.appliedVolts = 0;
    inputs.currentAmps = 0;
    inputs.tempCelsius = 0;
    inputs.reactionArmRotations = 0;
    inputs.reactionArmVoltage = 0;
  }

  @Override
  public void setHeight(Measure<Distance> height) {
    dispatcher.sendMotorInfo(Constants.CanIDs.ELEVATOR_CAN_ID, height.in(Units.Meter));
  }

  @Override
  public void setSensorPosition(Measure<Distance> position) {}

  @Override
  public void setReactionArmPosition(double rotations) {
    dispatcher.sendMotorInfo(
        Constants.CanIDs.REACTION_ARM_CAN_ID, Units.Rotations.of(rotations).in(Units.Radians));
  }

  @Override
  public void resetReactionArmPosition() {}
}
