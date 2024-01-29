package frc.robot.subsystems.visionGamepiece;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ProcessedGamepieceData {
  public Rotation2d targetYaw;
  public Rotation2d targetPitch;
  public double distance;
  public Pose2d pose;
  public Pose2d globalPose;
  public double timestamp_RIOFPGA_capture;

  public ProcessedGamepieceData(
      Rotation2d targetYaw,
      Rotation2d targetPitch,
      double distance,
      Pose2d pose,
      Pose2d globalPose,
      double timestamp_RIOFPGA_capture) {
    this.targetYaw = targetYaw;
    this.targetPitch = targetPitch;
    this.distance = distance;
    this.pose = pose;
    this.globalPose = globalPose;
    this.timestamp_RIOFPGA_capture = timestamp_RIOFPGA_capture;
  }
}
