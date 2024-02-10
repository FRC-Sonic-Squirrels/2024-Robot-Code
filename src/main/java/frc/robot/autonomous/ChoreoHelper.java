package frc.robot.autonomous;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.team2930.GeometryUtil;
import frc.robot.Constants;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class ChoreoHelper {

  private ChoreoTrajectory traj;
  private PIDController xFeedback;
  private PIDController yFeedback;
  private PIDController rotationalFeedback;
  private List<ChoreoTrajectoryState> states;
  private double initialTime;
  private Pose2d closestPose;

  /**
   * Helper class to go from timestamps of path to desired chassis speeds
   *
   * @param pathName name of path
   * @param translationalFeedback pid in x and y directions
   * @param rotationalFeedback pid for angular velocity
   */
  public ChoreoHelper(
      ChoreoTrajectory traj,
      PIDController translationalFeedback,
      PIDController rotationalFeedback,
      Pose2d initPose) {
    this.traj = traj;
    this.xFeedback = translationalFeedback;
    this.yFeedback = translationalFeedback;
    this.rotationalFeedback = rotationalFeedback;
    this.rotationalFeedback.enableContinuousInput(-Math.PI, Math.PI);
    states = getStates();
    ChoreoTrajectoryState closestState = null;
    for (int i = 0; i < traj.getPoses().length; i++) {
      closestState = calculateNewClosestState(closestState, states.get(i), initPose);
    }
    closestPose = closestState.getPose();
    initialTime = closestState.timestamp;
    Logger.recordOutput("Autonomous/closestPose", closestPose);
  }

  /**
   * Calculates field relative chassis speeds from path
   *
   * @param robotPose pose of the robot
   * @param timestamp time of path
   */
  public ChassisSpeeds calculateChassisSpeeds(Pose2d robotPose, double timestamp) {
    ChoreoTrajectoryState state = traj.sample(timestamp + initialTime, Constants.isRedAlliance());

    double xVel = state.velocityX + xFeedback.calculate(robotPose.getX(), state.x);
    double yVel = state.velocityY + yFeedback.calculate(robotPose.getY(), state.y);
    double omegaVel =
        state.angularVelocity
            + rotationalFeedback.calculate(robotPose.getRotation().getRadians(), state.heading);

    Logger.recordOutput("Autonomous/optimalPose", state.getPose());
    Logger.recordOutput(
        "Autonomous/desiredVelocity", new Pose2d(xVel, yVel, Rotation2d.fromRadians(omegaVel)));

    return new ChassisSpeeds(xVel, yVel, omegaVel);
  }

  private ChoreoTrajectoryState calculateNewClosestState(
      ChoreoTrajectoryState closestState, ChoreoTrajectoryState state, Pose2d pose) {
    if (closestState == null) {
      return state;
    }
    if (GeometryUtil.getDist(pose, state.getPose())
        <= GeometryUtil.getDist(pose, closestState.getPose())) {
      return state;
    } else {
      return closestState;
    }
  }

  private List<ChoreoTrajectoryState> getStates() {
    try {
      var f = traj.getClass().getDeclaredField("samples");
      f.setAccessible(true);
      var samples = (List<ChoreoTrajectoryState>) f.get(traj);
      return samples;
    } catch (NoSuchFieldException | IllegalAccessException e) {
      throw new RuntimeException(e);
    }
  }
}
