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

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.team6328.Alert;
import frc.lib.team6328.Alert.AlertType;
import frc.robot.configs.RobotConfig;
import frc.robot.configs.RobotConfig2023Rober;
import frc.robot.configs.RobotConfig2024;
import frc.robot.configs.SimulatorRobotConfig;
import java.util.function.Supplier;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  public static class RobotMode {
    private static final RobotType ROBOT = RobotType.ROBOT_SIMBOT;

    private static final Alert invalidRobotAlert =
        new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

    public static boolean isSimBot() {
      switch (getRobot()) {
        case ROBOT_SIMBOT:
        case ROBOT_SIMBOT_REAL_CAMERAS:
          return true;

        default:
          return false;
      }
    }

    // FIXME: update for various robots
    public static Mode getMode() {
      if (isSimBot()) {
        return Mode.SIM;
      }

      return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
    }

    public static RobotType getRobot() {
      if (RobotBase.isReal() && ROBOT == RobotType.ROBOT_SIMBOT) {
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_2024_MAESTRO;
      }

      return ROBOT;
    }

    // FIXME: update for various robots
    public enum RobotType {
      // use supplier because if we just create the object, the fields in the
      // config classes are also created. Meaning tunableNumber values are stuck to the first
      // object that is created. In this case ExampleRobotConfig. Suppliers solve this
      // by only creating the specific config object corresponding to the robot type
      ROBOT_SIMBOT(SimulatorRobotConfig::new),
      ROBOT_SIMBOT_REAL_CAMERAS(SimulatorRobotConfig::new),
      ROBOT_2023_RETIRED_ROBER(RobotConfig2023Rober::new),
      ROBOT_2024_MAESTRO(RobotConfig2024::new);

      public final Supplier<RobotConfig> config;

      RobotType(Supplier<RobotConfig> config) {
        this.config = config;
      }
    }

    public enum Mode {
      REAL,
      REPLAY,
      SIM
    }
  }

  public static double MAX_VOLTAGE = 12.0;

  public static class FieldConstants {
    // official Field dimensions
    // https://github.com/wpilibsuite/allwpilib/blob/1e168f363e23c42bde8b39e75765bb2eb81f97b2/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2024-crescendo.json#L292
    public static double FIELD_LENGTH = 16.451;
    public static double FIELD_WIDTH = .211;

    // FIXME: double check this number
    public static final Measure<Distance> SPEAKER_HEIGHT = Units.Meters.of(1.9812);
    public static final Translation2d BLUE_SPEAKER_TRANSLATION =
        new Translation2d(0.03950466960668564, 5.508944988250732);
    public static final Translation2d RED_SPEAKER_TRANSLATION =
        new Translation2d(16.508594512939453, 5.508944988250732);

    public static final Translation3d BLUE_SPEAKER_TRANSLATION_3D =
        new Translation3d(
            BLUE_SPEAKER_TRANSLATION.getX(),
            BLUE_SPEAKER_TRANSLATION.getY(),
            SPEAKER_HEIGHT.in(Units.Meters));
    public static final Translation3d RED_SPEAKER_TRANSLATION_3D =
        new Translation3d(
            RED_SPEAKER_TRANSLATION.getX(),
            RED_SPEAKER_TRANSLATION.getY(),
            SPEAKER_HEIGHT.in(Units.Meters));

    public static Translation2d getSpeakerTranslation() {
      return isRedAlliance() ? RED_SPEAKER_TRANSLATION : BLUE_SPEAKER_TRANSLATION;
    }

    public static Translation3d getSpeakerTranslation3D() {
      return isRedAlliance() ? RED_SPEAKER_TRANSLATION_3D : BLUE_SPEAKER_TRANSLATION_3D;
    }

    public static Measure<Distance> getDistanceToSpeaker(Pose2d pose) {
      var speakerTranslation = FieldConstants.getSpeakerTranslation();
      var speakerDx = speakerTranslation.getX() - pose.getX();
      var speakerDy = speakerTranslation.getY() - pose.getY();

      return Units.Meters.of(Math.hypot(speakerDx, speakerDy));
    }

    public static class Gamepieces {
      public static final Measure<Distance> NOTE_INNER_RADIUS = Units.Meters.of(0.127);
      public static final Measure<Distance> NOTE_OUTER_RADIUS = Units.Meters.of(0.1778);
    }
  }

  public static class MotorConstants {
    public static class KrakenConstants {
      public static final double MAX_RPM = 6000.0;
      public static final double NOMINAL_VOLTAGE_VOLTS = 12.0;
      public static final double STALL_TORQUE_NEWTON_METERS = 7.09;
      public static final double STALL_CURRENT_AMPS = 40.0;
      public static final double FREE_CURRENT_AMPS = 30.0;
      public static final double FREE_SPEED_RPM = 6000.0;
    }
  }

  public static class IntakeConstants {
    public static final double INTAKE_IDLE_PERCENT_OUT = 0.8;

    public static final double GEARING = 1.0;
    public static final double MOI = 5.0;
  }

  public static class EndEffectorConstants {
    public static final double INDEX_PERCENT_OUT = 0.8;

    public static final double GEARING = 1.0;
    public static final double MOI = 5.0;

    public static final double SHOOTING_PERCENT_OUT = 0.8;
  }

  public static class ElevatorConstants {
    // https://ss2930.sharepoint.com/:x:/s/Engineering/ETkKz1CrsINGj5Ia29ENxT4BE_Iqd_kAK_04iaW3kLqPuQ?clickparams=eyJBcHBOYW1lIjoiVGVhbXMtRGVza3RvcCIsIkFwcFZlcnNpb24iOiI0OS8yMzExMzAyODcyNCJ9
    public static final double GEAR_RATIO = 23.05;
    public static final double PULLEY_DIAMETER = 2.256;
    public static final double CARRIAGE_MASS = 10.0; // arbitrary
    public static final Measure<Distance> MAX_HEIGHT = Units.Inches.of(25.0);
    public static final Measure<Distance> TRUE_TOP_HARD_STOP = Units.Inches.of(26.5);

    public static final Measure<Distance> SAFE_HEIGHT = Units.Inches.of(3.0);
  }

  public static class ShooterConstants {
    public static final double PREP_RPM = 2500.0;
    public static final double SHOOTING_RPM = 5000.0;
    public static final double SHOOTING_PERCENT_OUT = 0.95;
    public static final Measure<Distance> SHOOTER_BASE_HEIGHT = Units.Inches.of(4.0);
    public static final Measure<Distance> SHOOTER_LENGTH = Units.Inches.of(12.0);

    public static final double SHOOTING_SPEED =
        SHOOTING_RPM / 60.0 * Launcher.WHEEL_DIAMETER.in(Units.Meters) * Math.PI;

    public static final double SHOOTING_TIME = 0.2;

    public static final Transform2d SHOOTER_OFFSET =
        new Transform2d(0, -Units.Inches.of(12).in(Units.Meters), new Rotation2d());

    public static final Translation3d SHOOTER_AXIS_OF_ROTATION =
        new Translation3d(
            SHOOTER_OFFSET.getX(), SHOOTER_OFFSET.getY(), Units.Inches.of(5).in(Units.Meters));

    public static class Pivot {
      // Old Calculations:
      // public static final Rotation2d DISTANCE_TO_SHOOTING_PITCH(double distanceMeters) {
      //   return new Rotation2d(
      //       Math.atan2(
      //           FieldConstants.SPEAKER_HEIGHT_METERS - SHOOTER_BASE_HEIGHT_METERS,
      // distanceMeters));
      // }

      // public static final double PITCH_VEL_RAD_PER_SEC(
      //     double velMetersPerSecond, double distanceMeters) {
      //   // velocity times derivative of distance to shooting pitch formula to get pitch velocity
      //   return velMetersPerSecond
      //       * -FieldConstants.SPEAKER_HEIGHT_METERS
      //       / (Math.pow(distanceMeters - SHOOTER_OFFSET_METERS.getY(), 2)
      //           + Math.pow(FieldConstants.SPEAKER_HEIGHT_METERS, 2));
      // }

      // public static final double GEARING = (40.0 / 12.0) * (40.0 / 20.0) * (120.0 / 10.0);
      public static final double GEARING = 125.0;

      public static final Rotation2d MIN_ANGLE_RAD = Rotation2d.fromDegrees(12.0);
      public static final Rotation2d MAX_ANGLE_RAD =
          Rotation2d.fromDegrees(59.0); // TRUE HARD STOP 61
      public static final Rotation2d HOME_POSITION = MIN_ANGLE_RAD;
      public static final Rotation2d TRUE_TOP_HARD_STOP = Rotation2d.fromDegrees(61.0);

      public static final Rotation2d SHOOTER_STOW_PITCH = Rotation2d.fromDegrees(14.0);

      public static final double SIM_INITIAL_ANGLE = Math.toRadians(14.0);
    }

    public static class Launcher {
      public static final double MOI = 5.0;
      // FIX ME: THIS VALUE HAS TO BE CONFIRMED
      public static final double GEARING = (18.0 / 30.0);
      public static final Measure<Distance> WHEEL_DIAMETER = Units.Inches.of(2.0);
    }

    public static class Kicker {
      public static final double MOI = 5.0;
      public static final double GEARING = 1.0;
      public static final double KICKING_PERCENT_OUT = 0.8;
    }
  }

  public static class LEDConstants {
    public static final int PWM_PORT = 8;
    public static final int MAX_LED_LENGTH = 60;
  }

  public static class CanIDs {
    // READ ME: CAN ID's THAT ARE NOT VALID TO USE
    // 1, 11, 21, 31
    // 2, 12, 22, 32
    // 3, 13, 23, 33
    // 4, 14, 24, 34
    // all these CAN ID's are reserved for the Drivetrain

    // TODO: get actual can ids
    public static final int INTAKE_CAN_ID = 34;

    public static final int SHOOTER_LEAD_CAN_ID = 33;
    public static final int SHOOTER_FOLLOW_CAN_ID = 36;
    public static final int SHOOTER_PIVOT_CAN_ID = 32;
    public static final int SHOOTER_KICKER_CAN_ID = 35;

    public static final int ARM_CAN_ID = 17;

    public static final int ELEVATOR_CAN_ID = 37;

    public static final int END_EFFECTOR_CAN_ID = 30;
    public static final int END_EFFECTOR_INTAKE_SIDE_TOF_CAN_ID = 35;
    public static final int END_EFFECTOR_SHOOTER_SIDE_TOF_CAN_ID = 36;
  }

  public static class DIOPorts {
    // TODO: get actual DIO ports
  }

  public enum ControlMode {
    POSITION,
    VELOCITY,
    VOLTAGE
  }

  public static class ArmConstants {
    public static final double GEAR_RATIO = (72.0 / 12.0) * (42.0 / 16.0);

    public static final Rotation2d MAX_ARM_ANGLE = Rotation2d.fromDegrees(90);
    public static final Rotation2d MIN_ARM_ANGLE = Rotation2d.fromDegrees(-90);
    public static final Rotation2d HOME_POSITION = MIN_ARM_ANGLE;

    public static final Rotation2d AMP_SAFE_ANGLE = Rotation2d.fromDegrees(-87);
  }

  public static class VisionGamepieceConstants {
    public static final Pose3d GAMEPIECE_CAMERA_POSE =
        new Pose3d(0.0, 0.0, Units.Inches.of(38).in(Units.Meters), new Rotation3d(0.0, 0.0, 0.0));
    public static final String CAMERA_NAME = RobotConfig2024.OBJECT_DETECTION_CAMERA_NAME;
  }

  public class AutoConstants {
    public static final double DIST_TO_START_INTAKING = 1.0;
  }
}
