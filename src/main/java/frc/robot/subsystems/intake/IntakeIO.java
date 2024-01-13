package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeIO {
  /** Contains all of the input data received from hardware. */
  public static class IntakeIOInputs implements LoggableInputs {
    public void toLog(LogTable table) {}

    public void fromLog(LogTable table) {}
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}
}
