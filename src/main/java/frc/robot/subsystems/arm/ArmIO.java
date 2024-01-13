package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ArmIO {
  /** Contains all of the input data received from hardware. */
  public static class ArmIOInputs implements LoggableInputs {
    public void toLog(LogTable table) {}

    public void fromLog(LogTable table) {}
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}
}
