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
  private double initialTime;
  private double timeOffset;
  private double allowedDist = 0.03;
  private double allowedExtraTime = 1.0;

  /**
   * Helper class to go from timestamps of path to desired chassis speeds
   *
   * @param traj trajectory to follow
   * @param translationalFeedback pid in x and y directions
   * @param rotationalFeedback pid for angular velocity
   */
  public ChoreoHelper(
      double initialTime,
      Pose2d initialPose,
      ChoreoTrajectory traj,
      PIDController translationalFeedbackX,
      PIDController translationalFeedbackY,
      PIDController rotationalFeedback) {
    this.traj = traj;
    this.xFeedback = translationalFeedbackX;
    this.yFeedback = translationalFeedbackY;
    this.rotationalFeedback = rotationalFeedback;
    this.rotationalFeedback.enableContinuousInput(-Math.PI, Math.PI);

    ChoreoTrajectoryState closestState = null;
    List<ChoreoTrajectoryState> states = getStates();
    for (int i = 0; i < traj.getPoses().length; i++) {
      closestState = calculateNewClosestState(closestState, states.get(i), initialPose);
    }
    if (closestState != null) {
      this.timeOffset = closestState.timestamp;
      Logger.recordOutput("Autonomous/closestPose", closestState.getPose());
    }
    this.initialTime = initialTime;
  }

  /**
   * Calculates field relative chassis speeds from path
   *
   * @param robotPose pose of the robot
   * @param timestamp time of path
   */
  public ChassisSpeeds calculateChassisSpeeds(Pose2d robotPose, double timestamp) {
    timestamp -= initialTime;
    timestamp += timeOffset;

    ChoreoTrajectoryState state = traj.sample(timestamp, Constants.isRedAlliance());

    Pose2d currentRobotPose = robotPose;

    double x = currentRobotPose.getX();
    double y = currentRobotPose.getY();
    double theta = currentRobotPose.getRotation().getRadians();

    double xVel = state.velocityX + xFeedback.calculate(x, state.x);
    double yVel = state.velocityY + yFeedback.calculate(y, state.y);

    Logger.recordOutput("Autonomous/stateLinearVel", Math.hypot(state.velocityX, state.velocityY));
    double omegaVel = state.angularVelocity + rotationalFeedback.calculate(theta, state.heading);

    Logger.recordOutput("Autonomous/optimalPose", state.getPose());
    Logger.recordOutput(
        "Autonomous/desiredVelocity", new Pose2d(xVel, yVel, Rotation2d.fromRadians(omegaVel)));

    if (GeometryUtil.getDist(currentRobotPose, traj.getFinalPose()) <= allowedDist
        || timestamp >= traj.getTotalTime() + allowedExtraTime) return null;

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
      //noinspection unchecked
      return (List<ChoreoTrajectoryState>) f.get(traj);
    } catch (NoSuchFieldException | IllegalAccessException e) {
      throw new RuntimeException(e);
    }
  }
}
