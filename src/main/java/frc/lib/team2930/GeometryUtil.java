package frc.lib.team2930;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeometryUtil {
  public static Rotation2d getHeading(Translation2d pose, Translation2d targetPose) {
    return new Rotation2d(targetPose.getX() - pose.getX(), targetPose.getY() - pose.getY());
  }
}
