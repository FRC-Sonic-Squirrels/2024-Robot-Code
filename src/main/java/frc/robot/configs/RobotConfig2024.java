package frc.robot.configs;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.swerve.SwerveModuleIOTalonFX;
import frc.robot.subsystems.vision.VisionModuleConfiguration;

public class RobotConfig2024 extends RobotConfig {

  private static final boolean PHOENIX_PRO_LICENSE = true;

  // TODO: ----- !IMPORTANT! FILL IN ALL VALUES !IMPORTANT! -----

  // ------------ SWERVE ---------------------
  private static final Measure<Distance> WHEEL_RADIUS = Units.Inches.of(2.0);

  // ------ SWERVE MODULE CONFIGURATIONS: CANID + OFFSET + INVERTS --------------
  // 0
  private static final IndividualSwerveModuleConfig FRONT_LEFT_MODULE_CONFIG =
      new IndividualSwerveModuleConfig(
          1,
          11,
          21,
          Rotation2d.fromRotations(0.197),
          InvertedValue.CounterClockwise_Positive,
          InvertedValue.Clockwise_Positive);
  // 1
  private static final IndividualSwerveModuleConfig FRONT_RIGHT_MODULE_CONFIG =
      new IndividualSwerveModuleConfig(
          2,
          12,
          22,
          Rotation2d.fromRotations(0.156),
          InvertedValue.CounterClockwise_Positive,
          InvertedValue.Clockwise_Positive);
  // 2
  private static final IndividualSwerveModuleConfig BACK_LEFT_MODULE_CONFIG =
      new IndividualSwerveModuleConfig(
          3,
          13,
          23,
          Rotation2d.fromRotations(-0.328),
          InvertedValue.CounterClockwise_Positive,
          InvertedValue.Clockwise_Positive);
  // 3
  private static final IndividualSwerveModuleConfig BACK_RIGHT_MODULE_CONFIG =
      new IndividualSwerveModuleConfig(
          4,
          14,
          24,
          Rotation2d.fromRotations(0.443),
          InvertedValue.CounterClockwise_Positive,
          InvertedValue.Clockwise_Positive);

  // -------- SWERVE CURRENT LIMITS ---------
  private static final CurrentLimitsConfigs DRIVE_TALON_CURRENT_LIMIT_CONFIGS =
      new CurrentLimitsConfigs()
          .withSupplyCurrentLimit(20)
          .withSupplyCurrentThreshold(30)
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
  // FIXE: RN copied from Mechanical advantage (6328) 2023 codebase. Should learn to tune
  // ourselves
  private static final LoggedTunableNumber ANGLE_KP = group.build("ANGLE_KP", 5.0);
  private static final LoggedTunableNumber ANGLE_KD = group.build("ANGLE_KD", 0.0);

  // ---------- SWERVE DRIVE MOTOR PID + KS + KV + KA CONSTANTS -------------
  private static final LoggedTunableNumber DRIVE_KP = group.build("DRIVE_KP", 0.0);
  private static final LoggedTunableNumber DRIVE_KD = group.build("DRIVE_KD", 0.0);

  private static final LoggedTunableNumber DRIVE_KS = group.build("DRIVE_KS", 0.0);
  private static final LoggedTunableNumber DRIVE_KV = group.build("DRIVE_KV", 0.13);
  private static final LoggedTunableNumber DRIVE_KA = group.build("DRIVE_KA", 1.6);

  // -------- GYRO CAN ID ---------
  private static final int GYRO_CAN_ID = 5;

  // -------- GYRO OFFSETS --------

  private static final double GYRO_MOUNTING_PITCH = 0.0;

  private static final double GYRO_MOUNTING_ROLL = -180.0;

  private static final double GYRO_MOUNTING_YAW = 90.0;

  // -------- CAN BUS NAME -----------
  private static final String CAN_BUS_NAME = "CANivore";

  // -------- ROBOT DIMENSIONS -----------
  // front to back
  private static final Measure<Distance> TRACK_WIDTH_X = Units.Inches.of(21.75);
  // left to right
  private static final Measure<Distance> TRACK_WIDTH_Y = Units.Inches.of(21.75);

  // ------- ROBOT MAX SPEED --------
  private static final double MAX_VELOCITY_METERS_PER_SECOND = 4.78;
  private static final double MAX_COAST_VELOCITY_METERS_PER_SECOND = 0.05;

  // ------- AUTONOMOUS CONSTANTS -------
  private static final LoggedTunableNumber AUTO_MAX_SPEED_METERS_PER_SECOND =
      group.build("AUTO_MAX_SPEED", 2.0);
  private static final LoggedTunableNumber AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED =
      group.build("AUTO_MAX_ACCEL", 2.0);

  private static final LoggedTunableNumber AUTO_TRANSLATION_KP =
      group.build("AUTO_TRANSLATION_KP", 0.2);
  private static final LoggedTunableNumber AUTO_TRANSLATION_KI =
      group.build("AUTO_TRANSLATION_KI", 0.0);
  private static final LoggedTunableNumber AUTO_TRANSLATION_KD =
      group.build("AUTO_TRANSLATION_KD", 0.0);
  private static final LoggedTunableNumber AUTO_THETA_KP = group.build("AUTO_THETA_KP", 1.0);
  private static final LoggedTunableNumber AUTO_THETA_KI = group.build("AUTO_THETA_KI", 0.0);
  private static final LoggedTunableNumber AUTO_THETA_KD = group.build("AUTO_THETA_KD", 0.0);

  // ---- VISION  -------
  public static final Transform3d SHOOTER_SIDE_LEFT =
      new Transform3d(
          new Translation3d(
              Units.Inches.of(-0.21682).in(Units.Meters),
              Units.Inches.of(6.7949).in(Units.Meters),
              Units.Inches.of(24.259267).in(Units.Meters)),
          new Rotation3d(Math.toRadians(180.0), Math.toRadians(-22.0), Math.toRadians(225.0)));

  public static final Transform3d SHOOTER_SIDE_RIGHT =
      new Transform3d(
          new Translation3d(
              Units.Inches.of(-0.21682).in(Units.Meters),
              Units.Inches.of(-6.7949).in(Units.Meters),
              Units.Inches.of(24.259267).in(Units.Meters)),
          new Rotation3d(Math.toRadians(180.0), Math.toRadians(-22.0), Math.toRadians(135.0)));

  public static final Transform3d INTAKE_SIDE_LEFT =
      new Transform3d(
          new Translation3d(
              Units.Inches.of(2.143069).in(Units.Meters),
              Units.Inches.of(12.127149).in(Units.Meters),
              Units.Inches.of(24.990728).in(Units.Meters)),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(-22.0), Math.toRadians(35.0)));

  public static final Transform3d INTAKE_SIDE_RIGHT =
      new Transform3d(
          new Translation3d(
              Units.Inches.of(2.143069).in(Units.Meters),
              Units.Inches.of(-12.127149).in(Units.Meters),
              Units.Inches.of(24.990728).in(Units.Meters)),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(-22.0), Math.toRadians(325.0)));

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

    return new SwerveModule[] {fl, fr, bl, br};
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

  // FIXME: define vision modules here
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
    return DRIVE_TALON_CURRENT_LIMIT_CONFIGS;
  }

  @Override
  public CurrentLimitsConfigs getSteerTalonCurrentLimitConfig() {
    return STEER_TALON_CURRENT_LIMIT_CONFIGS;
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
