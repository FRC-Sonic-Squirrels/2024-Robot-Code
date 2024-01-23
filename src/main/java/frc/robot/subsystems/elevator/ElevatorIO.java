package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ElevatorIOInputs {
    public double heightInches = 0.0;
    public double velInchesPerSecond = 0.0;
    public double voltage = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVel(double inchesPerSecond) {}

  public default void setHeight(double heightInches) {}

  public default void setPIDConstraints(double kP, double kD, double kG, Constraints constraints) {}
}
