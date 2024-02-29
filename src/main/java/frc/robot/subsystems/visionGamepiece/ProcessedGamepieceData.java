package frc.robot.subsystems.visionGamepiece;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import frc.robot.Constants.FieldConstants.Gamepieces;

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

  public double distance(Translation2d target) {
    return globalPose.getTranslation().minus(target).getNorm();
  }

  public boolean sameGamepiece(ProcessedGamepieceData gm) {
    return distance(gm.globalPose.getTranslation()) < Gamepieces.NOTE_TOLERANCE.in(Units.Meters);
  }

  public boolean isStale(double timestamp) {
    return (timestamp - timestamp_RIOFPGA_capture) > Gamepieces.NOTE_PERSISTENCE;
  }
}
