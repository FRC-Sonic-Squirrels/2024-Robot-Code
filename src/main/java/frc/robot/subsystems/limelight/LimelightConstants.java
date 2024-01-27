package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class LimelightConstants {
  public class gamepieces {
    public static final double NOTE_INNER_RADIUS_METERS = 0.254;
    public static final double NOTE_OUTER_RADIUS_METERS = 0.3048;
  }

  public class limelight {
    // TODO: GET LIMELIGHT POSE OF NEW ROBOT
    public static final Pose3d LIMELIGHT_POSE =
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(-2.7), Units.inchesToMeters(0.0), Units.inchesToMeters(33.42)),
            new Rotation3d(Math.toRadians(0.0), Math.toRadians(-5.2), Math.toRadians(180.0)));
  }
}
