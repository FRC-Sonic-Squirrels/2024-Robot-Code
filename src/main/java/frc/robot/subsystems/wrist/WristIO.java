package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class WristIOInputs {
    public Rotation2d angle = new Rotation2d();
    public double voltage = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void resetSensorPosition(Rotation2d angle) {}

  public default void setClosedLoopPosition(Rotation2d angle) {}

  public default void setClosedLoopConstants(
      double kP,
      double kD,
      double kG,
      double maxProfiledVelocity,
      double maxProfiledAcceleration) {}
}
