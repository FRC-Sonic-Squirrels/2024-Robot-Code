package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;
import frc.robot.subsystems.BaseInputs;

public interface ArmIO {
  /** Contains all of the input data received from hardware. */
  class Inputs extends BaseInputs {
    public Rotation2d armPosition = Constants.zeroRotation2d;
    public double armAngleDegrees;
    public double armAppliedVolts;
    public double armCurrentAmps;
    public double armTempCelsius;
    public double armVelocity;

    public Inputs(LoggerGroup logInputs) {
      super(logInputs);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Inputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void resetSensorPosition(Rotation2d angle) {}

  public default void setClosedLoopPosition(Rotation2d angle) {}

  public default void setClosedLoopConstants(
      double kP,
      double kD,
      double kG,
      double maxProfiledVelocity,
      double maxProfiledAcceleration) {}

  public default boolean setNeutralMode(NeutralModeValue value) {
    return false;
  }
}
