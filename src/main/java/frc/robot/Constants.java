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

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.team6328.Alert;
import frc.lib.team6328.Alert.AlertType;
import frc.robot.configs.ExampleRobotConfig;
import frc.robot.configs.RobotConfig;
import frc.robot.configs.RobotConfig2023Rober;
import frc.robot.configs.SimulatorRobotConfig;

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
    private static final RobotType ROBOT = RobotType.ROBOT_2023_RETIRED_ROBER;

    private static final Alert invalidRobotAlert =
        new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

    // FIXME: update for various robots
    public static Mode getMode() {
      switch (getRobot()) {
        case ROBOT_DEFAULT:
          return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

        case ROBOT_SIMBOT:
          return Mode.SIM;

        case ROBOT_2023_RETIRED_ROBER:
          return Mode.REAL;

        default:
          return Mode.REAL;
      }
    }

    public static RobotType getRobot() {
      if (RobotBase.isReal() && ROBOT == RobotType.ROBOT_SIMBOT) {
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_DEFAULT;
      }

      return ROBOT;
    }

    // FIXME: update for various robots
    public enum RobotType {
      // use supplier because if we just create the object, the fields in the
      // config classes are also created. Meaning tunableNumber values are stuck to the first
      // object that is created. In this case ExampleRobotConfig. Suppliers solve this
      // by only creating the specific config object coresponding to the robot type
      ROBOT_DEFAULT(() -> new ExampleRobotConfig()),
      ROBOT_SIMBOT(() -> new SimulatorRobotConfig()),
      ROBOT_2023_RETIRED_ROBER(() -> new RobotConfig2023Rober());

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
}
