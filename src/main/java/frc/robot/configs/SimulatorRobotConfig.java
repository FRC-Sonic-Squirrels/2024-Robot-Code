package frc.robot.configs;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.lib.constants.SwerveModuleConstants;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.subsystems.vision.VisionModuleConfiguration;

public class SimulatorRobotConfig extends RobotConfig {
  // private static final boolean PHOENIX_PRO_LICENSE = true;

  // ------------ SWERVE ---------------------
  private static final Measure<Distance> WHEEL_RADIUS = Units.Inches.of(2.0);

  // -------- GYRO OFFSETS --------

  private static final double GYRO_MOUNTING_PITCH = 0.0;

  private static final double GYRO_MOUNTING_ROLL = 0.0;

  private static final double GYRO_MOUNTING_YAW = 0.0;

  // ------ SWERVE MODULE CONFIGURATIONS: CANID + OFFSET + INVERTS --------------

  // --------- SWERVE GEAR RATIO ---------
  private static final double SWERVE_DRIVE_GEAR_RATIO =
      SwerveModuleConstants.MK4I.LEVEL_2_GEARING_DRIVE_GEAR_RATIO;
  private static final double SWERVE_STEER_GEAR_RATIO =
      SwerveModuleConstants.MK4I.GEARING_TURN_GEAR_RATIO;

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
  private static final Measure<Distance> TRACK_WIDTH_X = Units.Inches.of(30); // 24.375 inches
  // left to right
  private static final Measure<Distance> TRACK_WIDTH_Y = Units.Inches.of(30); // 22.625 inches

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
  public static final Transform3d SHOOTER_SIDE_LEFT =
      new Transform3d(
          new Translation3d(
              Units.Inches.of(-0.62273).in(Units.Meters),
              Units.Inches.of(9.625919).in(Units.Meters),
              Units.Inches.of(22.21467).in(Units.Meters)),
          new Rotation3d(0.0, 0.0, Math.toRadians(225.0)));

  public static final Transform3d SHOOTER_SIDE_RIGHT =
      new Transform3d(
          new Translation3d(
              Units.Inches.of(-0.62273).in(Units.Meters),
              Units.Inches.of(-9.625919).in(Units.Meters),
              Units.Inches.of(22.21467).in(Units.Meters)),
          new Rotation3d(0.0, 0.0, Math.toRadians(135.0)));

  public static final Transform3d INTAKE_SIDE_LEFT =
      new Transform3d(
          new Translation3d(
              Units.Inches.of(0.64738).in(Units.Meters),
              Units.Inches.of(13.14840).in(Units.Meters),
              Units.Inches.of(22.31306).in(Units.Meters)),
          new Rotation3d(0.0, 0.0, Math.toRadians(45.0)));

  public static final Transform3d INTAKE_SIDE_RIGHT =
      new Transform3d(
          new Translation3d(
              Units.Inches.of(0.64738).in(Units.Meters),
              Units.Inches.of(-13.14840).in(Units.Meters),
              Units.Inches.of(22.31306).in(Units.Meters)),
          new Rotation3d(0.0, 0.0, Math.toRadians(-45.0)));

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

    SwerveModule fl = new SwerveModule(0, this, new SwerveModuleIOSim());
    SwerveModule fr = new SwerveModule(1, this, new SwerveModuleIOSim());
    SwerveModule bl = new SwerveModule(2, this, new SwerveModuleIOSim());
    SwerveModule br = new SwerveModule(3, this, new SwerveModuleIOSim());

    SwerveModule[] modules = {fl, fr, bl, br};
    return modules;
  }

  @Override
  public SwerveModule[] getReplaySwerveModuleObjects() {
    return new SwerveModule[] {
      new SwerveModule(0, this, new SwerveModuleIO() {}),
      new SwerveModule(1, this, new SwerveModuleIO() {}),
      new SwerveModule(2, this, new SwerveModuleIO() {}),
      new SwerveModule(3, this, new SwerveModuleIO() {}),
    };
  }

  //
  // @Override
  // public VisionModuleConfiguration[] getVisionModuleObjects() {
  //   // does not work for SIM because we need to give VisionIOSim a pose supplier which can only
  // be
  //   // obtained from the drivetrain
  //   throw new RuntimeException("Unsupported action for SIM BOT");
  // }

  // FIXME: define vision modules here
  // CAVEAT: only call when we're testing real cameras on sim robot
  @Override
  public VisionModuleConfiguration[] getVisionModuleObjects() {
    return new VisionModuleConfiguration[] {
      VisionModuleConfiguration.build(INTAKE_SIDE_LEFT_CAMERA_NAME, INTAKE_SIDE_LEFT),
      VisionModuleConfiguration.build(INTAKE_SIDE_RIGHT_CAMERA_NAME, INTAKE_SIDE_RIGHT),
      VisionModuleConfiguration.build(SHOOTER_SIDE_LEFT_CAMERA_NAME, SHOOTER_SIDE_LEFT),
      VisionModuleConfiguration.build(SHOOTER_SIDE_RIGHT_CAMERA_NAME, SHOOTER_SIDE_RIGHT)
    };
  }

  @Override
  public VisionModuleConfiguration[] getReplayVisionModules() {
    return new VisionModuleConfiguration[] {
      VisionModuleConfiguration.buildReplayStub(INTAKE_SIDE_LEFT_CAMERA_NAME, INTAKE_SIDE_LEFT),
      VisionModuleConfiguration.buildReplayStub(INTAKE_SIDE_RIGHT_CAMERA_NAME, INTAKE_SIDE_RIGHT),
      VisionModuleConfiguration.buildReplayStub(SHOOTER_SIDE_LEFT_CAMERA_NAME, SHOOTER_SIDE_LEFT),
      VisionModuleConfiguration.buildReplayStub(SHOOTER_SIDE_RIGHT_CAMERA_NAME, SHOOTER_SIDE_RIGHT)
    };
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
  public Measure<Distance> getTrackWidth_Y() {
    return TRACK_WIDTH_Y;
  }

  @Override
  public Measure<Distance> getTrackWidth_X() {
    return TRACK_WIDTH_X;
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
  public PIDController getAutoTranslationPidController() {
    return new PIDController(
        AUTO_TRANSLATION_KP.get(), AUTO_TRANSLATION_KI.get(), AUTO_TRANSLATION_KD.get());
  }

  @Override
  public PIDController getAutoThetaPidController() {
    return new PIDController(AUTO_THETA_KP.get(), AUTO_THETA_KI.get(), AUTO_THETA_KD.get());
  }

  @Override
  public Measure<Distance> getWheelRadius() {
    return WHEEL_RADIUS;
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

  @Override
  public double getGyroMountingPitch() {
    return GYRO_MOUNTING_PITCH;
  }

  @Override
  public double getGyroMountingRoll() {
    return GYRO_MOUNTING_ROLL;
  }

  @Override
  public double getGyroMountingYaw() {
    return GYRO_MOUNTING_YAW;
  }
}
