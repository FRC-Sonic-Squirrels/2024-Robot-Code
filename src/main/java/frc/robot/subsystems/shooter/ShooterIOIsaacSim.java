package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.team2930.IsaacSimDispatcher;
import frc.robot.Constants;

public class ShooterIOIsaacSim implements ShooterIO {

  private IsaacSimDispatcher dispatcher;

  public ShooterIOIsaacSim(IsaacSimDispatcher dispatcher) {
    this.dispatcher = dispatcher;
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.pivotPosition =
        Rotation2d.fromRadians(dispatcher.recieveMotorPos(Constants.CanIDs.SHOOTER_PIVOT_CAN_ID));
    inputs.pivotVelocityRadsPerSec = 0.0;
    inputs.pivotAppliedVolts = 0.0;
    inputs.pivotCurrentAmps = 0.0;

    inputs.launcherRPM =
        new double[] {
          Units.radiansPerSecondToRotationsPerMinute(
              dispatcher.recieveMotorVel(Constants.CanIDs.SHOOTER_LEAD_CAN_ID)),
          Units.radiansPerSecondToRotationsPerMinute(
              dispatcher.recieveMotorVel(Constants.CanIDs.SHOOTER_FOLLOW_CAN_ID))
        };
    inputs.launcherAppliedVolts = new double[2];
    inputs.launcherCurrentAmps = new double[2];

    inputs.kickerAppliedVolts = 0.0;
    inputs.kickerCurrentAmps = 0.0;
    inputs.kickerVelocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(
            dispatcher.recieveMotorVel(Constants.CanIDs.SHOOTER_KICKER_CAN_ID));

    // launcher lead, launcher follower, pivot, kicker
    inputs.tempsCelcius = new double[4];

    inputs.timeOfFlightDistance = 18.0;
  }

  @Override
  public void setPivotPosition(Rotation2d rot) {
    dispatcher.sendMotorInfo(Constants.CanIDs.SHOOTER_PIVOT_CAN_ID, rot.getRadians());
  }

  @Override
  public void setPivotVoltage(double volts) {}

  @Override
  public void setLauncherVoltage(double volts) {
    dispatcher.sendMotorInfo(Constants.CanIDs.SHOOTER_PIVOT_CAN_ID, 0.0);
  }

  @Override
  public void setKickerVoltage(double volts) {
    dispatcher.sendMotorInfo(Constants.CanIDs.SHOOTER_KICKER_CAN_ID, 0.0);
  }

  @Override
  public void setKickerVelocity(double revPerMin) {
    dispatcher.sendMotorInfo(
        Constants.CanIDs.SHOOTER_KICKER_CAN_ID,
        Units.rotationsPerMinuteToRadiansPerSecond(revPerMin));
  }

  @Override
  public void setLauncherRPM(double topRollerRPM, double bottomRollerRPM) {
    dispatcher.sendMotorInfo(
        Constants.CanIDs.SHOOTER_LEAD_CAN_ID,
        Units.rotationsPerMinuteToRadiansPerSecond(topRollerRPM));
    dispatcher.sendMotorInfo(
        Constants.CanIDs.SHOOTER_FOLLOW_CAN_ID,
        Units.rotationsPerMinuteToRadiansPerSecond(bottomRollerRPM));
  }
}
