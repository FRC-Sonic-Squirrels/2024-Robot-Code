package frc.robot.subsystems.visionGamepiece;

public class RawGamepieceData {
  public double fiducialID;

  public double yaw;

  public double pitch;

  public double timestamp;

  public RawGamepieceData(double fiducialID, double yaw, double pitch, double timestamp) {
    this.fiducialID = fiducialID;
    this.yaw = yaw;
    this.pitch = pitch;
    this.timestamp = timestamp;
  }
}
