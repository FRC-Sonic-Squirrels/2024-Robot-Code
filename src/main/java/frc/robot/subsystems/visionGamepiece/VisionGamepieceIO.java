package frc.robot.subsystems.visionGamepiece;

import org.littletonrobotics.junction.AutoLog;

public interface VisionGamepieceIO {

  @AutoLog
  public static class VisionGamepieceIOInputs {
    public boolean isConnected = false;
    public boolean validTarget = false;
    public double totalLatencyMs = 0;
    public double timestamp = 0.0;
    public double[] pitch = new double[] {};
    public double[] yaw = new double[] {};
    public int[] id = new int[] {};
    public int targetCount = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionGamepieceIOInputs inputs) {}
}
