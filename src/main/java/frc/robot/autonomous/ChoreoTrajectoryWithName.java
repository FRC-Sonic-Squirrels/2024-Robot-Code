package frc.robot.autonomous;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.team2930.AllianceFlipUtil;

public record ChoreoTrajectoryWithName(String name, ChoreoTrajectory states) {
  public static ChoreoTrajectoryWithName getTrajectory(String name) {
    if (name == null) return null;
    return new ChoreoTrajectoryWithName(name, Choreo.getTrajectory(name));
  }

  public static String getName(ChoreoTrajectoryWithName trajWithName) {
    return trajWithName == null ? "NULL" : trajWithName.name();
  }

  public ChoreoTrajectoryWithName rescale(double speedScaling) {
    return new ChoreoTrajectoryWithName(name, ChoreoHelper.rescale(states, speedScaling));
  }

  public Pose2d getInitialPose(boolean flipForAlliance) {
    var pose = states.getInitialPose();

    return flipForAlliance ? AllianceFlipUtil.flipPoseForAlliance(pose) : pose;
  }

  public Pose2d getFinalPose(boolean flipForAlliance) {
    var pose = states.getFinalPose();

    return flipForAlliance ? AllianceFlipUtil.flipPoseForAlliance(pose) : pose;
  }
}
