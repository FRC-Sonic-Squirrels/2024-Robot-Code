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
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;

public class ChoreoHelper {
  private static final String ROOT_TABLE = "ChoreoHelper";

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Decimal log_stateLinearVel =
      logGroup.buildDecimal("stateLinearVel");
  private static final LoggerEntry.Decimal log_stateLinearVelError =
      logGroup.buildDecimal("stateLinearVelError");
  private static final LoggerEntry.Decimal log_stateScaleVel =
      logGroup.buildDecimal("stateScaleVel");
  private static final LoggerEntry.Decimal log_stateTimeOffset =
      logGroup.buildDecimal("stateTimeOffset");
  private static final LoggerEntry.Decimal log_stateTimestamp =
      logGroup.buildDecimal("stateTimestamp");
  private static final LoggerEntry.Struct<Pose2d> log_closestPose =
      logGroup.buildStruct(Pose2d.class, "closestPose");
  private static final LoggerEntry.Struct<Pose2d> log_optimalPose =
      logGroup.buildStruct(Pose2d.class, "optimalPose");
  private static final LoggerEntry.Struct<Pose2d> log_desiredVelocity =
      logGroup.buildStruct(Pose2d.class, "desiredVelocity");
  private static final LoggerEntry.Decimal log_distanceError =
      logGroup.buildDecimal("distanceError");
  private static final LoggerEntry.Decimal log_headingError = logGroup.buildDecimal("headingError");
  private static final LoggerEntry.Decimal log_pidXVelEffort =
      logGroup.buildDecimal("pidXVelEffort");
  private static final LoggerEntry.Decimal log_pidYVelEffort =
      logGroup.buildDecimal("pidYVelEffort");
  private static final LoggerEntry.Decimal log_pidVelEffort = logGroup.buildDecimal("pidVelEffort");

  private static final LoggerEntry.Bool log_isPaused = logGroup.buildBoolean("isPaused");

  private static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);
  private static final LoggedTunableNumber useCorrection = group.build("useCorrection", 1);

  private final ChoreoTrajectory traj;
  private final PIDController xFeedback;
  private final PIDController yFeedback;
  private final PIDController rotationalFeedback;
  private final double initialTime;
  private final double lagThreshold;
  private final double minVelToPause;

  private double timeOffset;
  private double pausedTime = Double.NaN;
  private ChoreoTrajectoryState stateTooBehind;

  /**
   * Helper class to go from timestamps of path to desired chassis speeds
   *
   * @param trajWithName trajectory to follow
   * @param translationalFeedbackX pid in x directions
   * @param translationalFeedbackY pid in y directions
   * @param rotationalFeedback pid for angular velocity
   */
  public ChoreoHelper(
      double initialTime,
      Pose2d initialPose,
      ChoreoTrajectoryWithName trajWithName,
      double lagThreshold,
      double minVelToPause,
      PIDController translationalFeedbackX,
      PIDController translationalFeedbackY,
      PIDController rotationalFeedback) {
    this.traj = trajWithName.states();
    this.lagThreshold = lagThreshold;
    this.minVelToPause = minVelToPause;
    this.xFeedback = translationalFeedbackX;
    this.yFeedback = translationalFeedbackY;
    this.rotationalFeedback = rotationalFeedback;
    this.rotationalFeedback.enableContinuousInput(-Math.PI, Math.PI);

    ChoreoTrajectoryState closestState = null;
    double closestDistance = Double.MAX_VALUE;
    double lastDistance = Double.MAX_VALUE;

    if (useCorrection.get() != 0) {
      for (ChoreoTrajectoryState state : getStates(traj)) {
        ChoreoTrajectoryState stateComputed =
            traj.sample(state.timestamp, Constants.isRedAlliance());
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
    }

    if (closestState != null) {
      this.timeOffset = closestState.timestamp;
      log_closestPose.info(closestState.getPose());
    }

    this.initialTime = initialTime;
  }

  public boolean isPaused() {
    return Double.isFinite(pausedTime);
  }

  public void pause(double timestamp) {
    if (!isPaused()) {
      pausedTime = timestamp;
    }
  }

  public void resume(double timestamp) {
    if ((isPaused())) {
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
    ChoreoTrajectoryState state;

    log_isPaused.info(isPaused());

    if (stateTooBehind != null) {
      state = stateTooBehind;
    } else {
      var timestampCorrected = timestamp - initialTime + timeOffset;

      state = traj.sample(timestampCorrected, Constants.isRedAlliance());
      if (timestampCorrected >= traj.getTotalTime()) {
        return null;
      }
    }

    while (true) {
      var lookaheadTime = Constants.kDefaultPeriod;

      var stateAhead = isFutureStateCloser(robotPose, state, lookaheadTime);
      if (stateAhead == null) break;

      timeOffset += lookaheadTime;
      state = stateAhead;
    }

    log_stateTimestamp.info(state.timestamp);
    log_stateTimeOffset.info(timeOffset);

    double xRobot = robotPose.getX();
    double yRobot = robotPose.getY();

    double xDesired = state.x;
    double yDesired = state.y;

    var distanceError = Math.hypot(xDesired - xRobot, yDesired - yRobot);

    log_distanceError.info(distanceError);

    boolean useCorrection = this.useCorrection.get() != 0;
    if (useCorrection) {
      if (distanceError < lagThreshold) {
        if (stateTooBehind != null) {
          stateTooBehind = null;
          resume(timestamp);
        }
      } else {
        var velMagnitude = Math.hypot(state.velocityX, state.velocityY);
        if (stateTooBehind == null && velMagnitude >= minVelToPause && state.timestamp > 0.25) {
          stateTooBehind = state;
          pause(timestamp);
        }
      }
    }

    double scaleVelocity = 1.0;

    if (Double.isFinite(pausedTime)) {
      if (stateTooBehind == null) {
        return null;
      }

      // If we are paused, scale down the velocity, to avoid fighting the Feedback PID.
      scaleVelocity *= Math.exp(-(timestamp - pausedTime));
    }

    log_stateScaleVel.info(scaleVelocity);

    double pidXVel = xFeedback.calculate(xRobot, xDesired);
    double pidYVel = yFeedback.calculate(yRobot, yDesired);

    double xVel = state.velocityX * scaleVelocity;
    double yVel = state.velocityY * scaleVelocity;

    if (useCorrection) {
      xVel += pidXVel;
      yVel += pidYVel;
    }

    log_pidXVelEffort.info(pidXVel);
    log_pidYVelEffort.info(pidYVel);
    log_pidVelEffort.info(Math.hypot(pidXVel, pidYVel));
    log_stateLinearVel.info(Math.hypot(state.velocityX, state.velocityY));
    log_stateLinearVelError.info(Math.hypot(state.velocityX - xVel, state.velocityY - yVel));

    Rotation2d rotation = robotPose.getRotation();
    double theta = rotation.getRadians();
    double omegaVel = state.angularVelocity + rotationalFeedback.calculate(theta, state.heading);

    log_optimalPose.info(state.getPose());
    log_desiredVelocity.info(new Pose2d(xVel, yVel, Rotation2d.fromRadians(omegaVel)));
    log_headingError.info(Math.toDegrees(GeometryUtil.optimizeRotation(theta - state.heading)));

    return ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xVel, yVel, omegaVel), rotation);
  }

  private ChoreoTrajectoryState isFutureStateCloser(
      Pose2d robotPose, ChoreoTrajectoryState state, double lookaheadTime) {
    var stateAhead = traj.sample(state.timestamp + lookaheadTime, Constants.isRedAlliance());

    double stateDistance = distanceToState(robotPose, state);
    double stateDistanceAhead = distanceToState(robotPose, stateAhead);
    return stateDistanceAhead < stateDistance ? stateAhead : null;
  }

  private static double distanceToState(Pose2d robotPose, ChoreoTrajectoryState state) {
    double xRobot = robotPose.getX();
    double yRobot = robotPose.getY();

    double xDesired = state.x;
    double yDesired = state.y;

    return Math.hypot(xDesired - xRobot, yDesired - yRobot);
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
