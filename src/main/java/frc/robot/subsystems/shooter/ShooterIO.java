package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ShooterIO {
  /** Contains all of the input data received from hardware. */
  public static class ShooterIOInputs implements LoggableInputs {
    public void toLog(LogTable table) {}

    public void fromLog(LogTable table) {}
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}
}
