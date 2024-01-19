package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {

  private SingleJointedArmSim arm =
      new SingleJointedArmSim(
          new DCMotor(
              Constants.MotorConstants.KrakenConstants.NOMINAL_VOLTAGE_VOLTS,
              Constants.MotorConstants.KrakenConstants.STALL_TORQUE_NEWTON_METERS,
              Constants.MotorConstants.KrakenConstants.STALL_CURRENT_AMPS,
              Constants.MotorConstants.KrakenConstants.FREE_CURRENT_AMPS,
              Constants.MotorConstants.KrakenConstants.FREE_SPEED_RPM / (Math.PI * 2),
              1),
          Constants.ShooterConstants.Pitch.GEARING,
          SingleJointedArmSim.estimateMOI(Units.feetToMeters(1.5), 20.0),
          Constants.ShooterConstants.SHOOTER_LENGTH,
          Constants.ShooterConstants.Pitch.MIN_ANGLE_RAD,
          Constants.ShooterConstants.Pitch.MAX_ANGLE_RAD,
          true,
          Constants.ShooterConstants.Pitch.SIM_INITIAL_ANGLE);

  private double targetVelRadPerSec = 0.0;
  private PIDController pitchController = new PIDController(0.01, 0, 0);

  public ShooterIOSim() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.pitch = new Rotation2d(arm.getAngleRads());
    arm.setInputVoltage(pitchController.calculate(arm.getVelocityRadPerSec(), targetVelRadPerSec));
  }

  @Override
  public void setVel(double radPerSecond) {
    targetVelRadPerSec = radPerSecond;
  }
}
