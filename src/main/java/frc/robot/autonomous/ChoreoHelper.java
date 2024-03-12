package frc.robot.autonomous;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;

public class ChoreoHelper {
  private static final String ROOT_TABLE = "ChoreoHelper";

  private static final LoggerGroup logGroup = new LoggerGroup(ROOT_TABLE);
  private static final LoggerEntry log_stateLinearVel = logGroup.build("stateLinearVel");
  private static final LoggerEntry log_closestPose = logGroup.build("closestPose");
  private static final LoggerEntry log_optimalPose = logGroup.build("optimalPose");
  private static final LoggerEntry log_desiredVelocity = logGroup.build("desiredVelocity");

  private final ChoreoTrajectory traj;
  private final PIDController xFeedback;
  private final PIDController yFeedback;
  private final PIDController rotationalFeedback;
  private final double initialTime;
  private double timeOffset;
  private double pausedTime = Double.NaN;

  /**
   * Helper class to go from timestamps of path to desired chassis speeds
   *
   * @param traj trajectory to follow
   * @param translationalFeedbackX pid in x directions
   * @param translationalFeedbackY pid in y directions
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
    double closestDistance = Double.MAX_VALUE;
    double lastDistance = Double.MAX_VALUE;

    for (ChoreoTrajectoryState state : getStates(traj)) {
      ChoreoTrajectoryState stateComputed = traj.sample(state.timestamp, Constants.isRedAlliance());
      double stateDistance = GeometryUtil.getDist(initialPose, stateComputed.getPose());

      if (stateDistance > lastDistance) {
        // Moving away, give up.
        break;
      }

      if (closestState == null || stateDistance < closestDistance) {
        closestState = state;
        closestDistance = stateDistance;
      }

      lastDistance = stateDistance;
    }

    if (closestState != null) {
      this.timeOffset = closestState.timestamp;
      log_closestPose.info(closestState.getPose());
    }

    this.initialTime = initialTime;
  }

  public void pause(double timestamp) {
    if (Double.isNaN(pausedTime)) {
      pausedTime = timestamp;
    }
  }

  public void resume(double timestamp) {
    if (!Double.isNaN(pausedTime)) {
      timeOffset += (pausedTime - timestamp);
      pausedTime = Double.NaN;
    }
  }

  /**
   * Calculates field relative chassis speeds from path
   *
   * @param robotPose pose of the robot
   * @param timestamp time of path
   */
  public ChassisSpeeds calculateChassisSpeeds(Pose2d robotPose, double timestamp) {
    if (!Double.isNaN(pausedTime)) {
      return null;
    }

    timestamp -= initialTime;
    timestamp += timeOffset;

    ChoreoTrajectoryState state = traj.sample(timestamp, Constants.isRedAlliance());

    double x = robotPose.getX();
    double y = robotPose.getY();
    Rotation2d rotation = robotPose.getRotation();

    double xVel = state.velocityX + xFeedback.calculate(x, state.x);
    double yVel = state.velocityY + yFeedback.calculate(y, state.y);

    log_stateLinearVel.info(Math.hypot(state.velocityX, state.velocityY));
    double theta = rotation.getRadians();
    double omegaVel = state.angularVelocity + rotationalFeedback.calculate(theta, state.heading);

    log_optimalPose.info(state.getPose());
    log_desiredVelocity.info(new Pose2d(xVel, yVel, Rotation2d.fromRadians(omegaVel)));

    if (timestamp > traj.getTotalTime()) return null;

    return ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xVel, yVel, omegaVel), rotation);
  }

  public static ChoreoTrajectory rescale(ChoreoTrajectory traj, double speedScaling) {
    if (speedScaling == 1.0) {
      return traj;
    }

    var newStates = new ArrayList<ChoreoTrajectoryState>();
    for (var state : getStates(traj)) {
      newStates.add(
          new ChoreoTrajectoryState(
              state.timestamp * speedScaling,
              state.x,
              state.y,
              state.heading,
              state.velocityX / speedScaling,
              state.velocityY / speedScaling,
              state.angularVelocity / speedScaling));
    }

    return new ChoreoTrajectory(newStates);
  }

  private static List<ChoreoTrajectoryState> getStates(ChoreoTrajectory traj) {
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
