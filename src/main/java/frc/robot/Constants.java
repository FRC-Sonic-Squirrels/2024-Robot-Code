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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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

  public class RobotMode {
    private static final RobotType ROBOT = RobotType.ROBOT_SIMBOT;

    private static final Alert invalidRobotAlert =
        new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

    // FIXME: update for various robots
    public static Mode getMode() {
      if (getRobot() == RobotType.ROBOT_SIMBOT) {
        return Mode.SIM;
      }

      return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
    }

    public static RobotType getRobot() {
      if (RobotBase.isReal() && ROBOT == RobotType.ROBOT_SIMBOT) {
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_2024;
      }

      return ROBOT;
    }

    // FIXME: update for various robots
    public enum RobotType {
      // use supplier because if we just create the object, the fields in the
      // config classes are also created. Meaning tunableNumber values are stuck to the first
      // object that is created. In this case ExampleRobotConfig. Suppliers solve this
      // by only creating the specific config object coresponding to the robot type
      ROBOT_SIMBOT(() -> new SimulatorRobotConfig()),
      ROBOT_2023_RETIRED_ROBER(() -> new RobotConfig2023Rober()),
      ROBOT_2024(() -> new RobotConfig2024());

      public Supplier<RobotConfig> config;

      private RobotType(Supplier<RobotConfig> config) {
        this.config = config;
      }
    }

    public enum Mode {
      REAL,
      REPLAY,
      SIM
    }
  }

  public class FieldConstants {
    // FIXME: double check this number
    public static double FIELD_LENGTH = 8.28347108459473 * 2.0;
    public static final double SPEAKER_HEIGHT_METERS = 1.9812;
  }

  public class MotorConstants {
    public class KrakenConstants {
      public static final double MAX_RPM = 6000.0;
      public static final double NOMINAL_VOLTAGE_VOLTS = 12.0;
      public static final double STALL_TORQUE_NEWTON_METERS = 7.09;
      public static final double STALL_CURRENT_AMPS = 40.0;
      public static final double FREE_CURRENT_AMPS = 30.0;
      public static final double FREE_SPEED_RPM = 6000.0;
    }
  }

  public class IntakeConstants {
    public static final double INTAKE_IDLE_PERCENT_OUT = 0.8;

    public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
    public static final double SUPPLY_CURRENT_LIMIT = 50.0;
    public static final double SUPPLY_CURRENT_THRESHOLD = 80.0;
    public static final double SUPPLY_TIME_THRESHOLD = 2.0;

    public static final double GEARING = 1.0;
    public static final double MOI = 5.0;
  }

  public class EndEffectorConstants {
    public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
    public static final double SUPPLY_CURRENT_LIMIT = 50.0;
    public static final double SUPPLY_CURRENT_THRESHOLD = 80.0;
    public static final double SUPPLY_TIME_THRESHOLD = 2.0;

    public static final double GEARING = 1.0;
    public static final double MOI = 5.0;
  }

  public class WristConstants {
    public static final Rotation2d MAX_WRIST_ANGLE = Rotation2d.fromDegrees(90);
    public static final Rotation2d MIN_WRIST_ANGLE = Rotation2d.fromDegrees(-90);
    public static final Rotation2d HOME_POSITION = Rotation2d.fromDegrees(-90);
    ;
  }

  public class ShooterConstants {
    public static final double PREP_RPM = 2500.0;
    public static final double SHOOTING_RPM = 5000.0;
    public static final double SHOOTER_OFFSET_METERS = 0.0;
    public static final double SHOOTER_LENGTH = Units.feetToMeters(1.5);

    public class Pitch {
      public static final Rotation2d DISTANCE_TO_SHOOTING_PITCH(double distanceMeters) {
        return new Rotation2d(
            Math.atan2(
                FieldConstants.SPEAKER_HEIGHT_METERS, distanceMeters - SHOOTER_OFFSET_METERS));
      }

      public static final double PITCH_VEL_RAD_PER_SEC(
          double velMetersPerSecond, double distanceMeters) {
        // velocity times derivative of distance to shooting pitch formula to get pitch velocity
        return velMetersPerSecond
            * -FieldConstants.SPEAKER_HEIGHT_METERS
            / (Math.pow(distanceMeters - SHOOTER_OFFSET_METERS, 2)
                + Math.pow(FieldConstants.SPEAKER_HEIGHT_METERS, 2));
      }

      public static final Rotation2d SHOOTER_STOW_PITCH = new Rotation2d(Math.toRadians(70.0));

      public static final double GEARING = 1.0;

      public static final double MIN_ANGLE_RAD = Math.toRadians(20.0);
      public static final double MAX_ANGLE_RAD = Math.toRadians(87.0);

      public static final double SIM_INITIAL_ANGLE = Math.toRadians(85);
    }
  }

  public class CanIDs {
    // TODO: get actual can ids
    public static final int INTAKE_CAN_ID = 0;
    public static final int SHOOTER_LEAD_CAN_ID = 1;
    public static final int SHOOTER_FOLLOW_CAN_ID = 2;
    public static final int ARM_CAN_ID = 3;
    public static final int ELEVATOR_LEAD_CAN_ID = 4;
    public static final int ELEVATOR_FOLLOW_CAN_ID = 5;
    public static final int END_EFFECTOR_CAN_ID = 6;
    public static final int WRIST_CAN_ID = 7;
  }

  public class DIOPorts {
    // TODO: get actual DIO ports
    public static final int INTAKE_BEAM_BREAK = 0;
    public static final int END_EFFECTOR_BEAM_BREAK = 0;
  }

  public enum ControlMode {
    POSITION,
    VELOCITY,
    VOLTAGE
  }

  public static final double MAX_VOLTAGE = 12.0;
}
