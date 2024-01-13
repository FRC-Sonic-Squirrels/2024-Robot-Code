package frc.robot.subsystems.limelight;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface LimelightIO {
  public static class LimelightIOInputs implements LoggableInputs {

    public boolean validTarget = false;
    public double pipelineLatencyMs = 0;
    public double captureLatencyMs = 0;
    public double totalLatencyMs = 0;
    public String json = "";

    public void toLog(LogTable table) {
      table.put("validTarget", validTarget);
      table.put("pipelineLatencyMs", pipelineLatencyMs);
      table.put("captureLatencyMs", captureLatencyMs);
      table.put("json", json);
    }

    public void fromLog(LogTable table) {
      validTarget = table.get("validTarget", validTarget);
      pipelineLatencyMs = table.get("pipelineLatencyMs", pipelineLatencyMs);
      captureLatencyMs = table.get("captureLatencyMs", captureLatencyMs);
      json = table.get("json", json);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LimelightIOInputs inputs) {}

  public default void ledMode(double mode) {}
}
