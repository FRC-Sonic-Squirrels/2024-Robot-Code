package frc.robot.subsystems.visionGamepiece;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.lib.team2930.GeometryUtil;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.Gamepieces;

public class ProcessedGamepieceData {
  private Rotation2d targetYaw;
  private Rotation2d targetPitch;
  private double distance;
  private Pose2d pose;
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

  public Pose2d getRobotCentricPose(Pose2d pose) {
    return globalPose.relativeTo(pose);
  }

  public Rotation2d getYaw(Pose2d pose) {
    Pose2d relativePose = getRobotCentricPose(pose);
    return Rotation2d.fromRadians(Math.atan2(relativePose.getY(), relativePose.getX()));
  }

  public Rotation2d getPitch(Pose2d pose) {
    double distance = getDistance(pose).in(Units.Meters);
    double height = Constants.VisionGamepieceConstants.GAMEPIECE_CAMERA_POSE.getZ();
    return Rotation2d.fromRadians(Math.atan2(height, distance));
  }

  public Measure<Distance> getDistance(Pose2d pose) {
    return Units.Meters.of(GeometryUtil.getDist(globalPose, pose));
  }

  public boolean sameGamepiece(ProcessedGamepieceData gm) {
    return getDistance(gm.globalPose).lt(Gamepieces.NOTE_TOLERANCE);
  }

  public boolean isStale(double timestamp) {
    return (timestamp - timestamp_RIOFPGA_capture) > Gamepieces.NOTE_PERSISTENCE;
  }
}
