package frc.robot.configs;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.constants.SwerveModuleConstants;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIOTalonFX;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionModuleConfiguration;

public class RobotConfig2024 extends RobotConfig {

  private static final boolean PHOENIX_PRO_LICENSE = true;

  // TODO: ----- !IMPORTANT! FILL IN ALL VALUES !IMPORTANT! -----

  // ------------ SWERVE ---------------------
  private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

  // ------ SWERVE MODULE CONFIGURATIONS: CANID + OFFSET + INVERTS --------------
  // 0
  private static final IndividualSwerveModuleConfig FRONT_LEFT_MODULE_CONFIG =
      IndividualSwerveModuleConfig.configureWithDefaultInverts(
          1, 11, 21, Rotation2d.fromDegrees(221.4));
  // 1
  private static final IndividualSwerveModuleConfig FRONT_RIGHT_MODULE_CONFIG =
      IndividualSwerveModuleConfig.configureWithDefaultInverts(
          2, 12, 22, Rotation2d.fromDegrees(190.6));
  // 2
  private static final IndividualSwerveModuleConfig BACK_LEFT_MODULE_CONFIG =
      IndividualSwerveModuleConfig.configureWithDefaultInverts(
          3, 13, 23, Rotation2d.fromDegrees(179.2));
  // 3
  private static final IndividualSwerveModuleConfig BACK_RIGHT_MODULE_CONFIG =
      IndividualSwerveModuleConfig.configureWithDefaultInverts(
          4, 14, 24, Rotation2d.fromDegrees(311.9));

  // -------- SWERVE CURRENT LIMITS ---------
  private static final CurrentLimitsConfigs DRIVE_TALON_CURRENT_LIMIT_CONFIGS =
      new CurrentLimitsConfigs()
          .withSupplyCurrentLimit(50)
          .withSupplyCurrentThreshold(60)
          .withSupplyTimeThreshold(0.1)
          .withSupplyCurrentLimitEnable(true);

  private static final CurrentLimitsConfigs STEER_TALON_CURRENT_LIMIT_CONFIGS =
      new CurrentLimitsConfigs()
          .withSupplyCurrentLimit(25)
          .withSupplyCurrentThreshold(40)
          .withSupplyTimeThreshold(0.1)
          .withSupplyCurrentLimitEnable(true);

  // --------- SWERVE GEAR RATIO ---------
  private static final double SWERVE_DRIVE_GEAR_RATIO =
      SwerveModuleConstants.MK4I.LEVEL_3_GEARING_DRIVE_GEAR_RATIO_PLUS_SPEED_KIT;
  private static final double SWERVE_STEER_GEAR_RATIO =
      SwerveModuleConstants.MK4I.GEARING_TURN_GEAR_RATIO;

  // ---------- SWERVE STEERING MOTOR PID CONSTANTS -----------
  private static final LoggedTunableNumber ANGLE_KP =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "ANGLE_KP", 9.609384164222876);
  private static final LoggedTunableNumber ANGLE_KD =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "ANGLE_KD", 0.28828152492668624);

  // ---------- SWERVE DRIVE MOTOR PID + KS + KV + KA CONSTANTS -------------
  private static final LoggedTunableNumber DRIVE_KP =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "DRIVE_KP", 0.2402346041055719);
  private static final LoggedTunableNumber DRIVE_KD =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "DRIVE_KD", 0.013212903225806451);

  private static final LoggedTunableNumber DRIVE_KS =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "DRIVE_KS", 0.25988);
  private static final LoggedTunableNumber DRIVE_KV =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "DRIVE_KV", 2.46330);
  private static final LoggedTunableNumber DRIVE_KA =
      new LoggedTunableNumber(LOGGED_TUNABLE_DASHBOARD_KEY + "DRIVE_KA", 0.12872);

  // -------- GYRO CAN ID ---------
  private static final int GYRO_CAN_ID = 18;

  // -------- CAN BUS NAME -----------
  private static final String CAN_BUS_NAME = "canbus1";

  // -------- ROBOT DIMENSIONS -----------
  // front to back
  private static final double TRACK_WIDTH_METERS_X = 0.619125; // 24.375 inches
  // left to right
  private static final double TRACK_WIDTH_METERS_Y = 0.574675; // 22.625 inches

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

  // ---- VISION  -------
  public static final Transform3d SHOOTER_SIDE_LEFT =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-0.62273),
              Units.inchesToMeters(9.625919),
              Units.inchesToMeters(22.21467)),
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(225.0)));

  public static final Transform3d SHOOTER_SIDE_RIGHT =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-0.62273),
              Units.inchesToMeters(-9.625919),
              Units.inchesToMeters(22.21467)),
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(135.0)));

  public static final Transform3d INTAKE_SIDE_LEFT =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(0.64738),
              Units.inchesToMeters(13.14840),
              Units.inchesToMeters(22.31306)),
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(45.0)));

  public static final Transform3d INTAKE_SIDE_RIGHT =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(0.64738),
              Units.inchesToMeters(-13.14840),
              Units.inchesToMeters(22.31306)),
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(-45.0)));

  public static final String OBJECT_DETECTION_CAMERA_NAME = "0_Object_Detection_ELP";
  public static final String SHOOTER_SIDE_LEFT_CAMERA_NAME = "1_Shooter_Left_See3Cam";
  public static final String SHOOTER_SIDE_RIGHT_CAMERA_NAME = "2_Shooter_Right_See3Cam";
  public static final String INTAKE_SIDE_LEFT_CAMERA_NAME = "3_Intake_Left_See3Cam";
  public static final String INTAKE_SIDE_RIGHT_CAMERA_NAME = "4_Intake_Right_See3Cam";

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
    SwerveModule fl =
        new SwerveModule(0, this, new SwerveModuleIOTalonFX(this, FRONT_LEFT_MODULE_CONFIG));
    SwerveModule fr =
        new SwerveModule(1, this, new SwerveModuleIOTalonFX(this, FRONT_RIGHT_MODULE_CONFIG));
    SwerveModule bl =
        new SwerveModule(2, this, new SwerveModuleIOTalonFX(this, BACK_LEFT_MODULE_CONFIG));
    SwerveModule br =
        new SwerveModule(3, this, new SwerveModuleIOTalonFX(this, BACK_RIGHT_MODULE_CONFIG));

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
    VisionModuleConfiguration intakeLeft =
        new VisionModuleConfiguration(
            new VisionIOPhotonVision(INTAKE_SIDE_LEFT_CAMERA_NAME),
            INTAKE_SIDE_LEFT_CAMERA_NAME,
            INTAKE_SIDE_LEFT);
    VisionModuleConfiguration intakeRight =
        new VisionModuleConfiguration(
            new VisionIOPhotonVision(INTAKE_SIDE_RIGHT_CAMERA_NAME),
            INTAKE_SIDE_RIGHT_CAMERA_NAME,
            INTAKE_SIDE_RIGHT);
    VisionModuleConfiguration shooterLeft =
        new VisionModuleConfiguration(
            new VisionIOPhotonVision(SHOOTER_SIDE_LEFT_CAMERA_NAME),
            SHOOTER_SIDE_LEFT_CAMERA_NAME,
            SHOOTER_SIDE_LEFT);
    VisionModuleConfiguration shooterRight =
        new VisionModuleConfiguration(
            new VisionIOPhotonVision(SHOOTER_SIDE_RIGHT_CAMERA_NAME),
            SHOOTER_SIDE_RIGHT_CAMERA_NAME,
            SHOOTER_SIDE_RIGHT);

    return new VisionModuleConfiguration[] {intakeLeft, intakeRight, shooterLeft, shooterRight};
  }

  @Override
  public VisionModuleConfiguration[] getReplayVisionModules() {
    VisionModuleConfiguration intakeLeft =
        new VisionModuleConfiguration(
            new VisionIO() {}, INTAKE_SIDE_LEFT_CAMERA_NAME, INTAKE_SIDE_LEFT);
    VisionModuleConfiguration intakeRight =
        new VisionModuleConfiguration(
            new VisionIO() {}, INTAKE_SIDE_RIGHT_CAMERA_NAME, INTAKE_SIDE_RIGHT);
    VisionModuleConfiguration shooterLeft =
        new VisionModuleConfiguration(
            new VisionIO() {}, SHOOTER_SIDE_LEFT_CAMERA_NAME, SHOOTER_SIDE_LEFT);
    VisionModuleConfiguration shooterRight =
        new VisionModuleConfiguration(
            new VisionIO() {}, SHOOTER_SIDE_RIGHT_CAMERA_NAME, SHOOTER_SIDE_RIGHT);

    return new VisionModuleConfiguration[] {intakeLeft, intakeRight, shooterLeft, shooterRight};
  }

  @Override
  public AprilTagFieldLayout getAprilTagFieldLayout() {
    return APRIL_TAG_FIELD.loadAprilTagLayoutField();
  }

  @Override
  public boolean getPhoenix6Licensed() {
    return PHOENIX_PRO_LICENSE;
  }

  @Override
  public IndividualSwerveModuleConfig[] getIndividualModuleConfigurations() {
    return new IndividualSwerveModuleConfig[] {
      FRONT_LEFT_MODULE_CONFIG,
      FRONT_RIGHT_MODULE_CONFIG,
      BACK_LEFT_MODULE_CONFIG,
      BACK_RIGHT_MODULE_CONFIG
    };
  }

  @Override
  public int getGyroCANID() {
    return GYRO_CAN_ID;
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
    return CAN_BUS_NAME;
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
  public PIDController getAutoTranslationPidController() {
    return new PIDController(
        AUTO_TRANSLATION_KP.get(), AUTO_TRANSLATION_KI.get(), AUTO_TRANSLATION_KD.get());
  }

  @Override
  public PIDController getAutoThetaPidController() {
    return new PIDController(AUTO_THETA_KP.get(), AUTO_THETA_KI.get(), AUTO_THETA_KD.get());
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
    return DRIVE_TALON_CURRENT_LIMIT_CONFIGS;
  }

  @Override
  public CurrentLimitsConfigs getSteerTalonCurrentLimitConfig() {
    return STEER_TALON_CURRENT_LIMIT_CONFIGS;
  }
}
