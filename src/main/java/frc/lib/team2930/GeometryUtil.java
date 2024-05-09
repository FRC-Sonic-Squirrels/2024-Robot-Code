package frc.lib.team2930;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;

public class GeometryUtil {
  public static Rotation2d getHeading(Translation2d pose, Translation2d targetPose) {
    return new Rotation2d(targetPose.getX() - pose.getX(), targetPose.getY() - pose.getY());
  }

  public static double getDist(Translation2d pose, Translation2d pose2) {
    return Math.hypot(pose.getX() - pose2.getX(), pose.getY() - pose2.getY());
  }

  public static double getDist(Pose2d pose, Pose2d pose2) {
    return Math.hypot(pose.getX() - pose2.getX(), pose.getY() - pose2.getY());
  }

  public static Translation3d translation2dTo3d(Translation2d translation) {
    return new Translation3d(translation.getX(), translation.getY(), 0.0);
  }

  public static Rotation3d rotation2dTo3d(Rotation2d rotation) {
    return new Rotation3d(0.0, 0.0, rotation.getRadians());
  }

  public static Translation3d translation3dFromMeasures(
      Measure<Distance> x, Measure<Distance> y, Measure<Distance> z) {
    return new Translation3d(x.in(Units.Meters), y.in(Units.Meters), z.in(Units.Meters));
  }

  public static boolean isPoseOutsideField(Pose2d pose) {
    double poseX = pose.getX();
    double poseY = pose.getY();
    if (poseX < 0.0 || poseY < 0.0 || poseX > 16.534 || poseY > 8.21) {
      return true;
    }
    return false;
  }

  /**
   * @param theta rotation measure radians
   * @return rotation between positive and negative PI radians
   */
  public static double optimizeRotation(double theta) {
    while (theta <= -Math.PI) theta += Math.PI * 2;
    while (theta >= Math.PI) theta -= Math.PI * 2;
    return theta;
  }

  public static double optimizeRotationInDegrees(double theta) {
    while (theta <= -180) theta += 360;
    while (theta >= 180) theta -= 360;
    return theta;
  }
}
