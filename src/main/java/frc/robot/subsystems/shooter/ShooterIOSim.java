package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterIOSim implements ShooterIO {

  private SingleJointedArmSim arm =
      new SingleJointedArmSim(
          new DCMotor(12.0, 7.09, 40.0, 30.0, 6000.0 / (Math.PI * 2), 1),
          1.0,
          SingleJointedArmSim.estimateMOI(Units.feetToMeters(1.5), 20.0),
          Units.feetToMeters(1.5),
          Math.toRadians(20.0),
          Math.toRadians(87.0),
          true,
          Math.toRadians(84.0));
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
