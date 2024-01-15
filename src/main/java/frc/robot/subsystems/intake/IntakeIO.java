package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeIO {
  /** Contains all of the input data received from hardware. */
  public static class IntakeIOInputs implements LoggableInputs {
    public double velocityRPM = 0.0;
    public double currentAmps = 0.0;
    public double deviceTemp = 0.0;

    public void toLog(LogTable table) {
      table.put("velocityRPM", velocityRPM);
      table.put("currentAmps", currentAmps);
      table.put("deviceTemp", deviceTemp);
    }

    public void fromLog(LogTable table) {
      velocityRPM = table.get("velocityRPM", velocityRPM);
      currentAmps = table.get("currentAmps", currentAmps);
      deviceTemp = table.get("deviceTemp", deviceTemp);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setPercentOut(double percent) {}
}
