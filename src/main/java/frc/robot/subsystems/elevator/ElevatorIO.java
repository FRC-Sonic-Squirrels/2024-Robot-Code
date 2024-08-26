package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public interface ElevatorIO {
  /** Contains all of the input data received from hardware. */
  public static class Inputs {
    public double heightInches = 0.0;
    public double velocityInchesPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double reactionArmRotations = 0.0;
    public double reactionArmVoltage = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Inputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setHeight(Measure<Distance> height) {}

  public default void setPIDConstraints(double kP, double kD, double kG, Constraints constraints) {}

  public default void setSensorPosition(Measure<Distance> position) {}

  public default boolean setNeutralMode(NeutralModeValue value) {
    return false;
  }

  public default void setReactionArmPosition(double rotations) {}

  public default void resetReactionArmPosition() {}

  public default boolean setReactionArmIdleMode(IdleMode idleMode) {
    return false;
  }
}
