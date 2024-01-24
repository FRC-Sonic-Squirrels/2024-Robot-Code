// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ArrayUtil;
import frc.lib.team6328.PoseEstimator;
import frc.lib.team6328.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {
  public static final Lock odometryLock = new ReentrantLock();

  private final GyroIO gyroIO;
  // private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR

  private final SwerveDriveKinematics kinematics;
  private Pose2d rawOdometryPose = new Pose2d();
  private Rotation2d lastGyroRotation = new Rotation2d();

  private final PoseEstimator poseEstimator;

  RobotConfig config;

  public Drivetrain(RobotConfig config, GyroIO gyroIO, SwerveModule[] swerveModules) {

    if (swerveModules.length > 4) {
      throw new RuntimeException("more than 4 modules given to drivetrain");
    }

    this.config = config;

    kinematics = config.getSwerveDriveKinematics();

    this.gyroIO = gyroIO;
    this.modules[0] = swerveModules[0];
    this.modules[1] = swerveModules[1];
    this.modules[2] = swerveModules[2];
    this.modules[3] = swerveModules[3];

    // FIXME: values copied from 6328, learn how to calculate these values
    poseEstimator = new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));

    // Configure AutoBuilder for PathPlanner
    // FIXME: pass in custom PID constants? Issue for this use case has been created:
    // https://github.com/mjansen4857/pathplanner/issues/474
    // FIXME: fix all the pathplanner jank
    // AutoBuilder.configureHolonomic(
    //   this::getPose,
    //   this::setPose,
    //   () -> kinematics.toChassisSpeeds(getModuleStates()),
    //   this::runVelocity,
    //   new HolonomicPathFollowerConfig(
    //     getMaxAngularSpeedRadPerSec(),
    //     getCharacterizationVelocity(),
    //     new ReplanningConfig()),
    //   () -> true,
    //   this);

    // // FIXME:
    // Pathfinding.setPathfinder(new LocalADStarAK());
    // PathPlannerLogging.setLogActivePathCallback(
    //     (activePath) -> {
    //       Logger.recordOutput(
    //           "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
    //     });
    // PathPlannerLogging.setLogTargetPoseCallback(
    //     (targetPose) -> {
    //       Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
    //     });
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    int deltaCount =
        gyroInputs.connected ? gyroInputs.odometryYawPositions.length : Integer.MAX_VALUE;
    for (int i = 0; i < 4; i++) {
      deltaCount = Math.min(deltaCount, modules[i].getPositionDeltas().length);
    }
    for (int deltaIndex = 0; deltaIndex < deltaCount; deltaIndex++) {
      // Read wheel deltas from each module
      SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        wheelDeltas[moduleIndex] = modules[moduleIndex].getPositionDeltas()[deltaIndex];
      }

      // The twist represents the motion of the robot since the last
      // sample in x, y, and theta based only on the modules, without
      // the gyro. The gyro is always disconnected in simulation.
      var twist = kinematics.toTwist2d(wheelDeltas);
      if (gyroInputs.connected) {
        // If the gyro is connected, replace the theta component of the twist
        // with the change in angle since the last sample.
        Rotation2d gyroRotation = gyroInputs.odometryYawPositions[deltaIndex];
        twist = new Twist2d(twist.dx, twist.dy, gyroRotation.minus(lastGyroRotation).getRadians());
        lastGyroRotation = gyroRotation;
      }
      // Apply the twist (change since last sample) to the current pose

      rawOdometryPose = rawOdometryPose.exp(twist);

      // fixme: all the modules should have the same timestamp so taking from 0 should be fine
      var timestamp = modules[0].getOdometryTimestamps()[deltaIndex];
      poseEstimator.addDriveData(timestamp, twist);
    }

    // FIXME:
    /*
     * Pending https://github.com/Mechanical-Advantage/AdvantageKit/issues/55
     * all @autoLogOutput fields need to be manually logged
     */
    Logger.recordOutput("Drivetrain/Pose", getRawOdometryPose());
    Logger.recordOutput("Drivetrain/PoseEstimatorPose", getPoseEstimatorPose());
    Logger.recordOutput("SwerveStates/Measured", getModuleStates());
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, config.getRobotMaxLinearVelocity());

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      // each translation forms a triangle from the center of the robot with the appropriate angle
      // for X stance
      headings[i] = config.getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public void addVisionEstimate(List<TimestampedVisionUpdate> visionData) {
    poseEstimator.addVisionData(visionData);
  }

  // FIXME: use pose estimator to get pose
  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/RobotOdometry")
  public Pose2d getRawOdometryPose() {
    return rawOdometryPose;
  }

  @AutoLogOutput(key = "Odometry/RobotEstimated")
  public Pose2d getPoseEstimatorPose() {
    return poseEstimator.getLatestPose();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return rawOdometryPose.getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    this.poseEstimator.resetPose(pose);
    this.rawOdometryPose = pose;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return config.getRobotMaxLinearVelocity();
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return config.getRobotMaxAngularVelocity();
  }

  public double[] getCurrentDrawAmps() {
    double[] current = new double[] {0};
    for (int i = 0; i < 4; i++) {
      current = ArrayUtil.concatWithArrayCopy(current, modules[i].getCurrentAmps());
    }
    return current;
  }
}
