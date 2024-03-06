package frc.robot.configs;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.SwerveModules;
import frc.robot.subsystems.vision.VisionModuleConfiguration;

public abstract class RobotConfig {

  protected static final TunableNumberGroup group = new TunableNumberGroup("RobotConfig");

  // VERY IMPORTANT
  public abstract boolean getPhoenix6Licensed();

  // drive PID + FeedForward constants. Not using kI
  public abstract LoggedTunableNumber getDriveKS();

  public abstract LoggedTunableNumber getDriveKV();

  public abstract LoggedTunableNumber getDriveKA();

  public abstract LoggedTunableNumber getDriveKP();

  public abstract LoggedTunableNumber getDriveKD();

  // angle PID constants. Not using kI
  public abstract LoggedTunableNumber getAngleKP();

  public abstract LoggedTunableNumber getAngleKD();

  // individual swerve module configs
  public abstract IndividualSwerveModuleConfig[] getIndividualModuleConfigurations();

  public abstract int getGyroCANID();

  public abstract double getGyroMountingPitch();

  public abstract double getGyroMountingRoll();

  public abstract double getGyroMountingYaw();

  public abstract String getCANBusName();

  // Robot MAX linear speeds
  public abstract double getRobotMaxLinearVelocity();

  public double getRobotMaxAngularVelocity() {
    return getRobotMaxLinearVelocity() / getDriveBaseRadius();
  }

  /**
   * the maximum velocity, in meters per second, at which the robot can be moving while disabled
   * before the drive motors are changed from brake to coast mode. Defaults to 0.
   */
  public abstract double getRobotMaxCoastVelocity();

  // -------------AUTONOMOUS-------------

  // Robot auto speeds
  public abstract LoggedTunableNumber getAutoMaxSpeed();

  public abstract LoggedTunableNumber getAutoMaxAcceleration();

  // auto translation PID
  public abstract PIDController getAutoTranslationPidController();

  // auto theta PID
  public abstract PIDController getAutoThetaPidController();

  // ----------------------- ROBOT DIMENSIONS -------------------

  public abstract Measure<Distance> getWheelRadius();

  public abstract double getSwerveModuleDriveGearRatio();

  public abstract double getSwerveModuleTurnGearRatio();

  /**
   * Returns the trackwidth (i.e., the center-to-center distance between the left and right wheels)
   * of the robot in meters. Must be overridden.
   *
   * @return the trackwidth (i.e., the center-to-center distance between the left and right wheels)
   *     of the robot in meters
   */
  public abstract Measure<Distance> getTrackWidth_Y();

  /**
   * Returns the wheelbase (i.e., the center-to-center distance between the front and back wheels)
   * of the robot in meters. Must be overridden.
   *
   * @return the wheelbase (i.e., the center-to-center distance between the front and back wheels)
   *     of the robot in meters
   */
  public abstract Measure<Distance> getTrackWidth_X();

  public double getDriveBaseRadius() {
    return Math.hypot(
        getTrackWidth_X().in(Units.Meters) / 2.0, getTrackWidth_Y().in(Units.Meters) / 2.0);
  }

  /**
   * Returns the swerve drive kinematics object for the robot. The geometry and coordinate systems
   * can be confusing. Refer to this document for a detailed explanation:
   * https://docs.google.com/document/d/17dg5cIfqVOlQTTbo2ust4QxTZlUoPNzuBu2oe58Ov84/edit#heading=h.x4ppzp81ed1
   *
   * @return the swerve drive kinematics object for the robot
   */

  // FIXME: is this correct?
  public SwerveDriveKinematics getSwerveDriveKinematics() {
    return new SwerveDriveKinematics(getModuleTranslations());
  }

  public Translation2d[] getModuleTranslations() {

    double widthX = getTrackWidth_X().in(Units.Meters) / 2.0;
    double widthY = getTrackWidth_Y().in(Units.Meters) / 2.0;
    return new Translation2d[] {
      // Front left
      new Translation2d(widthX, widthY),
      // Front right
      new Translation2d(widthX, -widthY),
      // Back left
      new Translation2d(-widthX, widthY),
      // Back right
      new Translation2d(-widthX, -widthY)
    };
  }

  // -------- CURRENT LIMITS -------------

  public abstract CurrentLimitsConfigs getDriveTalonCurrentLimitConfig();

  public abstract CurrentLimitsConfigs getSteerTalonCurrentLimitConfig();

  // -------- LIMELIGHT --------

  // public abstract Pose3d getlimelightPose();

  // public abstract double getNoteInnerRadiusMeters();

  // public abstract double getNoteOuterRadiusMeters();

  // --------- Build IO layers for subsystems -------------

  // return IO layers needed for subsystems
  public abstract SwerveModules getSwerveModuleObjects();

  public abstract SwerveModules getReplaySwerveModuleObjects();

  public abstract VisionModuleConfiguration[] getVisionModuleObjects();

  public abstract VisionModuleConfiguration[] getReplayVisionModules();

  public abstract AprilTagFieldLayout getAprilTagFieldLayout();
}
