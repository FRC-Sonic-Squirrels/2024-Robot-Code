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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.lib.team6328.PoseEstimator;
import frc.lib.team6328.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class Drivetrain extends SubsystemBase {
  public static final String ROOT_TABLE = "Drivetrain";

  private static final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);
  private static final ExecutionTiming timing_wait = new ExecutionTiming(ROOT_TABLE + "/wait");
  private static final ExecutionTiming timing_process =
      new ExecutionTiming(ROOT_TABLE + "/odometry");
  private static final ExecutionTiming timing_vision = new ExecutionTiming(ROOT_TABLE + "/vision");
  private static final ExecutionTiming timing_pose = new ExecutionTiming(ROOT_TABLE + "/pose");

  private static final LoggerGroup logGroupDrive = new LoggerGroup("Drive");
  private static final LoggerEntry logGyro = logGroupDrive.build("Gyro");

  private static final LoggerGroup logGroupSwerveStates = new LoggerGroup("SwerveStates");
  private static final LoggerEntry logSwerveStatesSetpoints =
      logGroupSwerveStates.build("Setpoints");
  private static final LoggerEntry logSwerveStatesSetpointsOptimized =
      logGroupSwerveStates.build("SetpointsOptimized");
  private static final LoggerEntry logSwerveStatesMeasured = logGroupSwerveStates.build("Measured");

  private static final LoggerGroup logGroupOdometryThread = new LoggerGroup("OdometryThread");
  private static final LoggerEntry logOdometryStatus =
      logGroupOdometryThread.build("status", SwerveModule.ODOMETRY_FREQUENCY);
  private static final LoggerEntry logOdometryTimestampSpread =
      logGroupOdometryThread.build("timestampSpread", SwerveModule.ODOMETRY_FREQUENCY);
  private static final LoggerEntry logOdometryTwist =
      logGroupOdometryThread.build("twist", SwerveModule.ODOMETRY_FREQUENCY);
  private static final LoggerEntry logOdometryCrash = logGroupOdometryThread.build("crash");

  private static final LoggerGroup logGroupDrivetrain = new LoggerGroup("Drivetrain");
  private static final LoggerEntry logDrivetrain_leftoverVelocity =
      logGroupDrivetrain.build("leftoverVelocity");
  private static final LoggerEntry logDrivetrain_speedsX = logGroupDrivetrain.build("speedsX");
  private static final LoggerEntry logDrivetrain_speedsY = logGroupDrivetrain.build("speedsY");
  private static final LoggerEntry logDrivetrain_linearSpeed =
      logGroupDrivetrain.build("linearSpeed");
  private static final LoggerEntry logDrivetrain_linearSpeedMax =
      logGroupDrivetrain.build("linearSpeedMax");
  private static final LoggerEntry logDrivetrain_speedsRot = logGroupDrivetrain.build("speedsRot");

  private static final LoggerGroup logGroupLocalization = new LoggerGroup("Localization");
  private static final LoggerEntry logLocalization_RobotPosition =
      logGroupLocalization.build("RobotPosition");
  private static final LoggerEntry logLocalization_RobotPosition_RAW_ODOMETRY =
      logGroupLocalization.build("RobotPosition_RAW_ODOMETRY");

  private static final LoggerGroup logGroupRobot = new LoggerGroup("Robot");
  private static final LoggerEntry log_FieldRelativeVel = logGroupRobot.build("FieldRelativeVel");

  public static final AutoLock odometryLock = new AutoLock("odometry", 100);

  public static final TunableNumberGroup group = new TunableNumberGroup("RobotConfig");
  public static final LoggedTunableNumber driveMotorAccelerationAuto =
      group.build("SwerveDRIVEAccelerationAuto", 1000);
  public static final LoggedTunableNumber driveMotorAccelerationTele =
      group.build("SwerveDRIVEAccelerationTele", 1000);

  private final RobotConfig config;
  private final Supplier<Boolean> isAutonomous;
  private final boolean isCANFD;

  private final GyroIO gyroIO;
  // private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final SwerveModules modules;

  private final SwerveDriveKinematics kinematics;
  private Pose2d rawOdometryPose = new Pose2d();

  private final PoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();
  private final Field2d rawOdometryField2d = new Field2d();

  public Drivetrain(
      RobotConfig config,
      GyroIO gyroIO,
      SwerveModules swerveModules,
      Supplier<Boolean> isAutonomous) {
    this.config = config;
    this.modules = swerveModules;
    this.gyroIO = gyroIO;
    this.isAutonomous = isAutonomous;

    String canBusName = config.getCANBusName();
    if (canBusName != null) {
      isCANFD = com.ctre.phoenix6.CANBus.isNetworkFD(canBusName);
    } else {
      isCANFD = false;
    }

    kinematics = config.getSwerveDriveKinematics();

    // FIXME: values copied from 6328, learn how to calculate these values
    poseEstimator = new PoseEstimator(0.4, 0.4, 0.3);

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

    var thread = new Thread(this::runOdometry);
    thread.setName("PhoenixOdometryThread");
    thread.setDaemon(true);
    thread.start();
  }

  public void periodic() {
    try (var ignored = timing.start()) {
      gyroIO.updateInputs(gyroInputs);
      modules.updateInputs();

      logGyro.info(gyroInputs);
      modules.periodic();

      // Stop moving when disabled
      if (DriverStation.isDisabled()) {
        modules.stop();
      }
      // Log empty setpoint states when disabled
      if (DriverStation.isDisabled()) {
        logSwerveStatesSetpoints.info(new SwerveModuleState[] {});
        logSwerveStatesSetpointsOptimized.info(new SwerveModuleState[] {});
      }

      logSwerveStatesMeasured.info(getModuleStates());

      log_FieldRelativeVel.info(getFieldRelativeVelocities());
      logLocalization_RobotPosition.info(getPoseEstimatorPose());
      logLocalization_RobotPosition_RAW_ODOMETRY.info(rawOdometryPose);

      field2d.setRobotPose(getPoseEstimatorPose());
      SmartDashboard.putData("Localization/field2d", field2d);

      rawOdometryField2d.setRobotPose(rawOdometryPose);
      SmartDashboard.putData("Localization/rawOdometryField2d", rawOdometryField2d);
    }
  }

  private void runOdometry() {
    List<BaseStatusSignal> signals = new ArrayList<>();
    gyroIO.registerSignalForOdometry(signals);
    modules.registerSignalForOdometry(signals);

    var signalsArray = signals.toArray(new BaseStatusSignal[0]);
    Rotation2d lastGyroRotation = null;
    StatusCode lastStatusCode = null;

    while (true) {
      try {
        double timestamp;

        // Wait for updates from all signals
        if (isCANFD) {
          StatusCode statusCode;

          try (var ignored = timing_wait.start()) {
            statusCode =
                BaseStatusSignal.waitForAll(2.0 / SwerveModule.ODOMETRY_FREQUENCY, signalsArray);
          }

          if (statusCode != lastStatusCode) {
            logOdometryStatus.info(statusCode);
            lastStatusCode = statusCode;
          }

          if (statusCode != StatusCode.OK) {
            continue;
          }

          double timeSum = 0.0;
          double timeMin = Double.MAX_VALUE;
          double timeMax = 0.0;

          for (BaseStatusSignal s : signals) {
            var time = s.getTimestamp().getTime();
            timeSum += time;
            timeMin = Math.min(timeMin, time);
            timeMax = Math.max(timeMax, time);
          }
          timestamp = timeSum / signalsArray.length;
          var timestampSpread =
              Math.max(Math.abs(timeMax - timestamp), Math.abs(timeMin - timestamp));

          logOdometryTimestampSpread.info(timestampSpread);
        } else {
          // "waitForAll" does not support blocking on multiple
          // signals with a bus that is not CAN FD, regardless
          // of Pro licensing. No reasoning for this behavior
          // is provided by the documentation.
          Thread.sleep((long) (1000.0 / SwerveModule.ODOMETRY_FREQUENCY));
          BaseStatusSignal.refreshAll(signalsArray);

          timestamp = Utils.getCurrentTimeSeconds();
        }

        var gyroRotation = gyroIO.updateOdometry(gyroInputs);
        var wheelDeltas = modules.updateOdometry();

        // The twist represents the motion of the robot since the last
        // sample in x, y, and theta based only on the modules, without
        // the gyro. The gyro is always disconnected in simulation.
        var twist = kinematics.toTwist2d(wheelDeltas);

        logOdometryTwist.info(twist);
        if (gyroRotation != null) {
          if (lastGyroRotation != null) {
            var dtheta = gyroRotation.minus(lastGyroRotation).getRadians();
            twist = new Twist2d(twist.dx, twist.dy, dtheta);
          }

          lastGyroRotation = gyroRotation;
        }

        try (var ignored2 = odometryLock.lock()) // Prevents odometry updates while reading data
        {
          try (var ignored3 = timing_process.start()) {
            // Apply the twist (change since last sample) to the current pose
            rawOdometryPose = rawOdometryPose.exp(twist);

            poseEstimator.addDriveData(timestamp, twist);
          }
        }
      } catch (Exception e) {
        logOdometryCrash.info(e.toString());
      }
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds, boolean prioritizeRotation) {
    if (prioritizeRotation) {
      // Calculate module setpoints

      SwerveModuleState[] justRotationSetpointStates =
          kinematics.toSwerveModuleStates(
              new ChassisSpeeds(0.0, 0.0, speeds.omegaRadiansPerSecond));

      SwerveModuleState[] calculatedSetpointStates = kinematics.toSwerveModuleStates(speeds);

      double realMaxSpeed = 0;
      for (SwerveModuleState moduleState : calculatedSetpointStates) {
        realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
      }
      if (realMaxSpeed > config.getRobotMaxLinearVelocity()) {
        /*

        linear cannot exceed the leftover amount after rotation
        Vmax - abs(Vr) = hypot(Vx, Vy)

        preserve xy ratio
        Vyprev      Vy
        -------- = ----
        Vxprev      Vx

        solve system of equations for Vx and Vy

        Vx = Vxprev / hypot(Vxprev, Vyprev) * abs(max(0.0, Vmax - abs(Vr)))

        Vy = Vyprev * Vx / Vxprev

        */

        double rotationSpeedMetersPerSecond = justRotationSetpointStates[0].speedMetersPerSecond;
        double vxMetersPerSecond =
            (speeds.vxMetersPerSecond
                    / Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond))
                * Math.abs(
                    Math.max(
                        0.0,
                        config.getRobotMaxLinearVelocity()
                            - Math.abs(rotationSpeedMetersPerSecond)));

        // double vxMetersPerSecond =
        //     Math.copySign(
        //         Math.abs(
        //                 Math.max(
        //                     0.0,
        //                     config.getRobotMaxLinearVelocity()
        //                         - Math.abs(justRotationSetpointStates[0].speedMetersPerSecond)))
        //             / Math.sqrt(
        //                 (1.0 + Math.pow(speeds.vyMetersPerSecond / speeds.vxMetersPerSecond,
        // 2.0))),
        //         speeds.vxMetersPerSecond);
        double vyMetersPerSecond =
            speeds.vyMetersPerSecond * vxMetersPerSecond / speeds.vxMetersPerSecond;

        speeds =
            new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, speeds.omegaRadiansPerSecond);

        logDrivetrain_leftoverVelocity.info(
            config.getRobotMaxLinearVelocity() - rotationSpeedMetersPerSecond);
      }
    }

    logDrivetrain_speedsX.info(speeds.vxMetersPerSecond);
    logDrivetrain_speedsY.info(speeds.vyMetersPerSecond);
    logDrivetrain_linearSpeed.info(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    logDrivetrain_linearSpeedMax.info(config.getRobotMaxLinearVelocity());
    logDrivetrain_speedsRot.info(speeds.omegaRadiansPerSecond);

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, config.getRobotMaxLinearVelocity());

    // Send setpoints to modules
    // The module returns the optimized state, useful for logging

    double driveMotorAcceleration;
    if (isAutonomous.get()) {
      driveMotorAcceleration = driveMotorAccelerationAuto.get();
    } else {
      driveMotorAcceleration = driveMotorAccelerationTele.get();
    }

    var optimizedSetpointStates = modules.runSetpoints(setpointStates, driveMotorAcceleration);

    // Log setpoint states
    logSwerveStatesSetpoints.info(setpointStates);
    logSwerveStatesSetpointsOptimized.info(optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds(), false);
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
    modules.runCharacterizationVolts(volts);
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return modules.getCharacterizationVelocity();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  private SwerveModuleState[] getModuleStates() {
    return modules.getModuleStates();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public Pose2d getFieldRelativeVelocities() {
    Translation2d translation =
        new Translation2d(
                getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond)
            .rotateBy(getRawOdometryPose().getRotation());
    return new Pose2d(translation, new Rotation2d(getChassisSpeeds().omegaRadiansPerSecond));
  }

  public void addVisionEstimate(List<TimestampedVisionUpdate> visionData) {
    try (var ignored = odometryLock.lock()) // Prevents odometry updates while reading data
    {
      try (var ignored2 = timing_vision.start()) {
        poseEstimator.addVisionData(visionData);
      }
    }
  }

  /**
   * WARNING - THIS IS THE RAW *ODOMETRY* POSE, THIS DOES NOT ACCOUNT FOR VISION DATA & SHOULD
   * EXCLUSIVELY BE USED FOR LOGGING AND ANALYSIS
   */
  private Pose2d getRawOdometryPose() {
    return rawOdometryPose;
  }

  public Pose2d getPoseEstimatorPose() {
    try (var ignored = odometryLock.lock()) // Prevents odometry updates while reading data
    {
      try (var ignored2 = timing_pose.start()) {
        return poseEstimator.getLatestPose();
      }
    }
  }

  // public Rotation2d getRotation() {
  //   return getPoseEstimatorPose().getRotation();
  // }

  public Rotation2d getRotationGyroOnly() {
    return rawOdometryPose.getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    try (var ignored = odometryLock.lock()) // Prevents odometry updates while reading data
    {
      this.poseEstimator.resetPose(pose, Utils.getCurrentTimeSeconds() + 0.2);
      this.rawOdometryPose = pose;
    }
  }

  public void setRawOdometryPose(Pose2d pose) {
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
    return modules.getCurrentDrawAmps();
  }
}
