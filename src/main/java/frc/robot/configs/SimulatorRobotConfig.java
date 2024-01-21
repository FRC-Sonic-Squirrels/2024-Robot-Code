package frc.robot.configs;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.constants.SwerveModuleConstants;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionModuleConfiguration;

public class SimulatorRobotConfig extends RobotConfig {
  // private static final boolean PHOENIX_PRO_LICENSE = true;

  // ------------ SWERVE ---------------------
  private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

  // ------ SWERVE MODULE CONFIGURATIONS: CANID + OFFSET + INVERTS --------------

  // --------- SWERVE GEAR RATIO ---------
  private static final double SWERVE_DRIVE_GEAR_RATIO =
      SwerveModuleConstants.MK4I.LEVEL_2_GEARING_DRIVE_GEAR_RATIO;
  private static final double SWERVE_STEER_GEAR_RATIO =
      SwerveModuleConstants.MK4I.LEVEL_2_GEARING_TURN_GEAR_RATIO;

  // ---------- SWERVE STEERING MOTOR PID CONSTANTS -----------
  private static final LoggedTunableNumber ANGLE_KP =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "ANGLE_KP", 10.0);
  private static final LoggedTunableNumber ANGLE_KD =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "ANGLE_KD", 0.0);

  // ---------- SWERVE DRIVE MOTOR PID + KS + KV + KA CONSTANTS -------------
  private static final LoggedTunableNumber DRIVE_KP =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "DRIVE_KP", 0.1);
  private static final LoggedTunableNumber DRIVE_KD =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "DRIVE_KD", 0.0);

  private static final LoggedTunableNumber DRIVE_KS =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "DRIVE_KS", 0.0);
  private static final LoggedTunableNumber DRIVE_KV =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "DRIVE_KV", 0.13);
  private static final LoggedTunableNumber DRIVE_KA =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "DRIVE_KA", 0.0);

  // -------- ROBOT DIMENSIONS -----------
  // front to back
  private static final double TRACK_WIDTH_METERS_X = Units.inchesToMeters(30); // 24.375 inches
  // left to right
  private static final double TRACK_WIDTH_METERS_Y = Units.inchesToMeters(30); // 22.625 inches

  // ------- ROBOT MAX SPEED --------
  private static final double MAX_VELOCITY_METERS_PER_SECOND = 4.78;
  private static final double MAX_COAST_VELOCITY_METERS_PER_SECOND = 0.05;

  // ------- AUTONOMOUS CONSTANTS -------
  private static final LoggedTunableNumber AUTO_MAX_SPEED_METERS_PER_SECOND =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "AUTO_MAX_SPEED", 2.0);
  private static final LoggedTunableNumber AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "AUTO_MAX_ACCEL", 2.0);

  private static final LoggedTunableNumber AUTO_TRANSLATION_KP =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "AUTO_TRANSLATION_KP", 6.0);
  private static final LoggedTunableNumber AUTO_TRANSLATION_KI =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "AUTO_TRANSLATION_KI", 0.0);
  private static final LoggedTunableNumber AUTO_TRANSLATION_KD =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "AUTO_TRANSLATION_KD", 0.0);
  private static final LoggedTunableNumber AUTO_THETA_KP =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "AUTO_THETA_KP", 4.9);
  private static final LoggedTunableNumber AUTO_THETA_KI =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "AUTO_THETA_KI", 0.0);
  private static final LoggedTunableNumber AUTO_THETA_KD =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "AUTO_THETA_KD", 0.0);

  // ---- VISION CAMERA TRANSFORM3d's -------

  // public so that robot container can access
  // SIM is a special case because we need to access drivetrain.getPose()
  // this is not possible in robot configs so we must construct the vision modules in robot
  // container
  public static final Transform3d FRONT_LEFT_ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-0.51), Units.inchesToMeters(10.2), Units.inchesToMeters(22.8)),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(30.0)));

  public static final Transform3d FRONT_RIGHT_ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-0.51), Units.inchesToMeters(-10.2), Units.inchesToMeters(22.8)),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(-30.0)));

  public static final Transform3d BACK_ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-2.7), Units.inchesToMeters(0.0), Units.inchesToMeters(33.42)),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(10.0), Math.toRadians(180.0)));

  public static final String FRONT_LEFT_CAMERA_NAME = "LeftCamera";
  public static final String FRONT_RIGHT_CAMERA_NAME = "RightCamera";
  public static final String BACK_CAMERA_NAME = "BackCamera";

  public static final AprilTagFields APRIL_TAG_FIELD = AprilTagFields.k2024Crescendo;

  /*
   *
   *
   * FUNCTIONS RETURNING CONSTANTS:
   *
   * YOU MUST CHANGED 2 METHODS:
   * getSwerveModuleObjects()
   * getVisionModuleObjects()
   *
   */

  // FIXME: define your swerve module objects here
  @Override
  public SwerveModule[] getSwerveModuleObjects() {

    SwerveModule fl = new SwerveModule(0, this, new SwerveModuleIOSim());
    SwerveModule fr = new SwerveModule(1, this, new SwerveModuleIOSim());
    SwerveModule bl = new SwerveModule(2, this, new SwerveModuleIOSim());
    SwerveModule br = new SwerveModule(3, this, new SwerveModuleIOSim());

    SwerveModule[] modules = {fl, fr, bl, br};
    return modules;
  }

  @Override
  public SwerveModule[] getReplaySwerveModuleObjects() {
    SwerveModule[] modules = {
      new SwerveModule(0, this, new SwerveModuleIO() {}),
      new SwerveModule(1, this, new SwerveModuleIO() {}),
      new SwerveModule(2, this, new SwerveModuleIO() {}),
      new SwerveModule(3, this, new SwerveModuleIO() {}),
    };

    return modules;
  }

  // FIXME: define vision modules here
  @Override
  public VisionModuleConfiguration[] getVisionModuleObjects() {
    // does not work for SIM because we need to give VisionIOSim a pose supplier which can only be
    // obtained from the drivetrain
    throw new RuntimeException("Unsupported action for SIM BOT");
    // VisionModule frontLeft =
    //     new VisionModule(new VisionIO() {}, "frontLeft", FRONT_LEFT_ROBOT_TO_CAMERA);
    // VisionModule frontRight =
    //     new VisionModule(new VisionIO() {}, "frontRight", FRONT_RIGHT_ROBOT_TO_CAMERA);
    // VisionModule back = new VisionModule(new VisionIO() {}, "back", BACK_ROBOT_TO_CAMERA);

    // return new VisionModule[] {frontLeft, frontRight, back};
  }

  @Override
  public VisionModuleConfiguration[] getReplayVisionModules() {
    VisionModuleConfiguration frontLeft =
        new VisionModuleConfiguration(new VisionIO() {}, "frontLeft", FRONT_LEFT_ROBOT_TO_CAMERA);
    VisionModuleConfiguration frontRight =
        new VisionModuleConfiguration(new VisionIO() {}, "frontRight", FRONT_RIGHT_ROBOT_TO_CAMERA);
    VisionModuleConfiguration back =
        new VisionModuleConfiguration(new VisionIO() {}, "back", BACK_ROBOT_TO_CAMERA);

    return new VisionModuleConfiguration[] {frontLeft, frontRight, back};
  }

  @Override
  public AprilTagFieldLayout getAprilTagFieldLayout() {
    return APRIL_TAG_FIELD.loadAprilTagLayoutField();
  }

  @Override
  public boolean getPhoenix6Licensed() {
    throw new RuntimeException("Unsupported action for SIM BOT");
  }

  @Override
  public IndividualSwerveModuleConfig[] getIndividualModuleConfigurations() {
    throw new RuntimeException("Unsupported action for SIM BOT");
  }

  @Override
  public int getGyroCANID() {
    throw new RuntimeException("Unsupported action for SIM BOT");
  }

  @Override
  public double getTrackWidth_Y() {
    return TRACK_WIDTH_METERS_Y;
  }

  @Override
  public double getTrackWidth_X() {
    return TRACK_WIDTH_METERS_X;
  }

  @Override
  public double getRobotMaxLinearVelocity() {
    return MAX_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public double getRobotMaxCoastVelocity() {
    return MAX_COAST_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public String getCANBusName() {
    throw new RuntimeException("Unsupported action for SIM BOT");
  }

  @Override
  public LoggedTunableNumber getDriveKS() {
    return DRIVE_KS;
  }

  @Override
  public LoggedTunableNumber getDriveKV() {
    return DRIVE_KV;
  }

  @Override
  public LoggedTunableNumber getDriveKA() {
    return DRIVE_KA;
  }

  @Override
  public LoggedTunableNumber getDriveKP() {
    return DRIVE_KP;
  }

  @Override
  public LoggedTunableNumber getDriveKD() {
    return DRIVE_KD;
  }

  @Override
  public LoggedTunableNumber getAngleKP() {
    return ANGLE_KP;
  }

  @Override
  public LoggedTunableNumber getAngleKD() {
    return ANGLE_KD;
  }

  @Override
  public LoggedTunableNumber getAutoMaxSpeed() {
    return AUTO_MAX_SPEED_METERS_PER_SECOND;
  }

  @Override
  public LoggedTunableNumber getAutoMaxAcceleration() {
    return AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
  }

  @Override
  public LoggedTunableNumber getAutoTranslationKP() {
    return AUTO_TRANSLATION_KP;
  }

  @Override
  public LoggedTunableNumber getAutoTranslationKI() {
    return AUTO_TRANSLATION_KI;
  }

  @Override
  public LoggedTunableNumber getAutoTranslationKD() {
    return AUTO_TRANSLATION_KD;
  }

  @Override
  public LoggedTunableNumber getAutoThetaKP() {
    return AUTO_THETA_KP;
  }

  @Override
  public LoggedTunableNumber getAutoThetaKI() {
    return AUTO_THETA_KI;
  }

  @Override
  public LoggedTunableNumber getAutoThetaKD() {
    return AUTO_THETA_KD;
  }

  @Override
  public double getWheelRadius() {
    return WHEEL_RADIUS_METERS;
  }

  @Override
  public double getSwerveModuleDriveGearRatio() {
    return SWERVE_DRIVE_GEAR_RATIO;
  }

  @Override
  public double getSwerveModuleTurnGearRatio() {
    return SWERVE_STEER_GEAR_RATIO;
  }

  @Override
  public CurrentLimitsConfigs getDriveTalonCurrentLimitConfig() {
    throw new RuntimeException("Unsupported action for SIM BOT");
  }

  @Override
  public CurrentLimitsConfigs getSteerTalonCurrentLimitConfig() {
    throw new RuntimeException("Unsupported action for SIM BOT");
  }
}
