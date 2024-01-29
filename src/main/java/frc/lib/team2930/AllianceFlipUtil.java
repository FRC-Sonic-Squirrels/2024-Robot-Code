package frc.lib.team2930;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class AllianceFlipUtil {

  public static Translation2d flipVelocitiesForAlliance(Translation2d originalVelocity) {
    if (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue) {
      return originalVelocity;
    }

    return originalVelocity.unaryMinus();
  }

  public static Translation2d mirrorTranslation2DOverCenterLine(Translation2d originalTranslation) {
    return new Translation2d(
        Constants.FieldConstants.FIELD_LENGTH - originalTranslation.getX(),
        originalTranslation.getY());
  }

  public static Pose2d mirrorPose2DOverCenterLine(Pose2d originalPose) {
    return new Pose2d(
        Constants.FieldConstants.FIELD_LENGTH - originalPose.getX(),
        originalPose.getY(),
        new Rotation2d(-originalPose.getRotation().getCos(), originalPose.getRotation().getSin()));
  }
}