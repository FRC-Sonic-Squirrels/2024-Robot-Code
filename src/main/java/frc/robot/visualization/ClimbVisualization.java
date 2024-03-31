package frc.robot.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;

public class ClimbVisualization {
  private static final LoggerGroup logGroup = LoggerGroup.build("AutoClimb");
  private static final LoggerEntry.Struct<Pose3d> logPredictedPose3d =
      logGroup.buildStruct(Pose3d.class, "predictedPose3d");

  private static final ClimbVisualization instance = new ClimbVisualization();
  private double additionalRobotHeight = 0.0;
  private Pose3d pose = Constants.zeroPose3d;

  public static ClimbVisualization getInstance() {
    return instance;
  }

  public void updateAdditionalHeight(double additionalRobotHeight) {
    this.additionalRobotHeight = additionalRobotHeight;
  }

  public void updateVisualization(Pose2d robotPose) {
    pose =
        new Pose3d(
            robotPose.getX(),
            robotPose.getY(),
            Units.inchesToMeters(additionalRobotHeight),
            new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians()));
  }

  public void logPose() {
    logPredictedPose3d.info(pose);
  }
}
