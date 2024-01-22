package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;

public class WristIOReal implements WristIO {
  public WristIOReal() {}

  @Override
  public void updateInputs(WristIOInputs inputs) {}

  @Override
  public void setVoltage(double volts) {}

  @Override
  public void resetSensorPosition(Rotation2d angle) {}

  @Override
  public void setClosedLoopPosition(Rotation2d angle) {}

  @Override
  public void setClosedLoopConstants(
      double kP,
      double kD,
      double kG,
      double maxProfiledVelocity,
      double maxProfiledAcceleration) {}
}
