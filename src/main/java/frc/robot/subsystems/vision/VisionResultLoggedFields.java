package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public record VisionResultLoggedFields(
    VisionResultStatus status,
    double numSeenTargets,
    double tagAmbiguity,
    double averageDistanceFromTags,
    double distanceFromExistingPoseEstimate,
    double xyStandardDeviation,
    double thetaStandardDeviation,
    Pose3d robotPose3d) {

  public static VisionResultLoggedFields unsuccessfulResult(VisionResultStatus status) {
    return new VisionResultLoggedFields(status);
  }

  private VisionResultLoggedFields(VisionResultStatus status) {
    this(status, -1, -1, -1, -1, -1, -1, new Pose3d());
  }
}
