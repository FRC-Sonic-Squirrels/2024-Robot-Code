package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ArmIOInputs {
    public Rotation2d armPosition = new Rotation2d();
    public double armAppliedVolts;
    public double armCurrentAmps;
    public double armTempCelsius;
    public double armVelocity;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void resetSensorPosition(Rotation2d angle) {}

  public default void setClosedLoopPosition(Rotation2d angle) {}

  public default void setClosedLoopConstants(
      double kP,
      double kD,
      double kG,
      double maxProfiledVelocity,
      double maxProfiledAcceleration) {}

  public default void setNeutralMode(NeutralModeValue value) {}
}
