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
import frc.lib.team2930.AutoLock;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team6328.LoggedTunableNumber;
import frc.lib.team6328.PoseEstimator;
import frc.lib.team6328.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {
  public static final AutoLock odometryLock = new AutoLock();

  public static final LoggedTunableNumber driveMotorAccelerationAuto =
      new LoggedTunableNumber("RobotConfig/SwerveDRIVEAccelerationAuto", 1000);

  public static final LoggedTunableNumber driveMotorAccelerationTele =
      new LoggedTunableNumber("RobotConfig/SwerveDRIVEAccelerationTele", 1000);

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
    poseEstimator = new PoseEstimator(0.1, 0.1, 0.3);

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
    try (var ignored = new ExecutionTiming("Drivetrain")) {
      gyroIO.updateInputs(gyroInputs);
      modules.updateInputs();

      Logger.processInputs("Drive/Gyro", gyroInputs);
      modules.periodic();

      // Stop moving when disabled
      if (DriverStation.isDisabled()) {
        modules.stop();
      }
      // Log empty setpoint states when disabled
      if (DriverStation.isDisabled()) {
        Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
        Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
      }

      Logger.recordOutput("SwerveStates/Measured", getModuleStates());

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
          var statusCode =
              BaseStatusSignal.waitForAll(2.0 / SwerveModule.ODOMETRY_FREQUENCY, signalsArray);

          if (statusCode != lastStatusCode) {
            Logger.recordOutput("OdometryThread/status", statusCode);
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
          Logger.recordOutput("OdometryThread/timestampSpread", timestampSpread);

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

        Logger.recordOutput("OdometryThread/twist", twist);
        if (gyroRotation != null) {
          if (lastGyroRotation != null) {
            var dtheta = gyroRotation.minus(lastGyroRotation).getRadians();
            twist = new Twist2d(twist.dx, twist.dy, dtheta);
          }

          lastGyroRotation = gyroRotation;
        }

        try (var ignored2 = odometryLock.lock()) // Prevents odometry updates while reading data
        {
          // Apply the twist (change since last sample) to the current pose
          rawOdometryPose = rawOdometryPose.exp(twist);

          poseEstimator.addDriveData(timestamp, twist);
        }
      } catch (Exception e) {
        Logger.recordOutput("OdometryThread/crash", e.toString());
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
        Logger.recordOutput(
            "Drivetrain/leftoverVelocity",
            config.getRobotMaxLinearVelocity() - rotationSpeedMetersPerSecond);
      }
    }

    Logger.recordOutput("Drivetrain/speedsX", speeds.vxMetersPerSecond);
    Logger.recordOutput("Drivetrain/speedsY", speeds.vyMetersPerSecond);
    Logger.recordOutput(
        "Drivetrain/linearSpeed", Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    Logger.recordOutput("Drivetrain/linearSpeedMax", config.getRobotMaxLinearVelocity());
    Logger.recordOutput("Drivetrain/speedsRot", speeds.omegaRadiansPerSecond);

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
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
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
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    return modules.getModuleStates();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  @AutoLogOutput(key = "Robot/FieldRelativeVel")
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
      poseEstimator.addVisionData(visionData);
    }
  }

  /**
   * WARNING - THIS IS THE RAW *ODOMETRY* POSE, THIS DOES NOT ACCOUNT FOR VISION DATA & SHOULD
   * EXCLUSIVELY BE USED FOR LOGGING AND ANALYSIS
   */
  @AutoLogOutput(key = "Localization/RobotPosition_RAW_ODOMETRY")
  private Pose2d getRawOdometryPose() {
    return rawOdometryPose;
  }

  @AutoLogOutput(key = "Localization/RobotPosition")
  public Pose2d getPoseEstimatorPose() {
    try (var ignored = odometryLock.lock()) // Prevents odometry updates while reading data
    {
      return poseEstimator.getLatestPose();
    }
  }

  public Rotation2d getRotation() {
    return getPoseEstimatorPose().getRotation();
  }

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
