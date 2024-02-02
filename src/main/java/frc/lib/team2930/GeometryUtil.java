package frc.lib.team2930;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class GeometryUtil {
  public static Rotation2d getHeading(Translation2d pose, Translation2d targetPose) {
    return new Rotation2d(targetPose.getX() - pose.getX(), targetPose.getY() - pose.getY());
  }

  public static double getDist(Translation2d pose, Translation2d pose2) {
    return Math.hypot(pose.getX() - pose2.getX(), pose.getY() - pose2.getY());
  }

  public static Translation3d translation2dTo3d(Translation2d translation) {
    return new Translation3d(translation.getX(), translation.getY(), 0.0);
  }

  public static Rotation3d rotation2dTo3d(Rotation2d rotation) {
    return new Rotation3d(0.0, 0.0, rotation.getRadians());
  }
}
