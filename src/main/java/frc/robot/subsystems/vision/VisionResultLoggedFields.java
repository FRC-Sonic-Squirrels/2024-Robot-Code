package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;

public record VisionResultLoggedFields(
    VisionResultStatus status,
    int numSeenTargets,
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
    this(status, -1, -1, -1, -1, -1, -1, Constants.zeroPose3d);
  }
}
