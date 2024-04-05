package frc.robot.subsystems.visionGamepiece;

public interface VisionGamepieceIO {

  public static class Inputs {
    public boolean isConnected = false;
    public boolean validTarget = false;
    public double totalLatencyMs = 0;
    public double timestamp = 0.0;
    public double[] pitch = new double[] {};
    public double[] yaw = new double[] {};
    public double[] area = new double[] {};
    public int targetCount = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Inputs inputs) {}

  public default void setPipelineIndex(int index) {}
}
