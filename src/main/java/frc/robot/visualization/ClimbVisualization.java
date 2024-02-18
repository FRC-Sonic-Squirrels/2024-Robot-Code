package frc.robot.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class ClimbVisualization {

  private static ClimbVisualization instance = new ClimbVisualization();
  private double additionalRobotHeight = 0.0;
  private Pose3d pose = new Pose3d();

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
    Logger.recordOutput("AutoClimb/predictedPose3d", pose);
  }
}
