package frc.robot.subsystems.visionGamepiece;

public class RawGamepieceData {
  public double yaw;

  public double pitch;

  public double timestamp;

  public RawGamepieceData(double yaw, double pitch, double timestamp) {
    this.yaw = yaw;
    this.pitch = pitch;
    this.timestamp = timestamp;
  }
}
