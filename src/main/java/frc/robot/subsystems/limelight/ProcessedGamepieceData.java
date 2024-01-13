package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ProcessedGamepieceData {
  public Rotation2d targetYaw;
  public Rotation2d targetPitch;
  public double distance;
  public Pose2d pose;
  public double timestamp_RIOFPGA_capture;
  public double confidence;

  public ProcessedGamepieceData(
      Rotation2d targetYaw,
      Rotation2d targetPitch,
      double distance,
      Pose2d pose,
      double timestamp_RIOFPGA_capture,
      double confidence) {
    this.targetYaw = targetYaw;
    this.targetPitch = targetPitch;
    this.distance = distance;
    this.pose = pose;
    this.timestamp_RIOFPGA_capture = timestamp_RIOFPGA_capture;
    this.confidence = confidence;
  }
}
