package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ArmIOInputs {
    public double armPositionRad;
    public double armAppliedVolts;
    public double armCurrentAmps;
    public double armTempCelsius;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setClosedLoopPosition(Rotation2d angle) {}

  public default void setClosedLoopConstants(
      double kP, double kD, double maxProfiledVelocity, double maxProfiledAcceleration) {}
}
