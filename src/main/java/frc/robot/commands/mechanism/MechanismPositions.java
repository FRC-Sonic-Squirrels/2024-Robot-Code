package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class MechanismPositions {
  private static LoggedTunableNumber homeElevatorHeightInches =
      new LoggedTunableNumber("MechanismPositions/homeElevatorHeightInches", 8.0);
  private static LoggedTunableNumber homeArmAngleDegrees =
      new LoggedTunableNumber("MechanismPositions/homeArmAngleDegrees", -90.0);

  private static LoggedTunableNumber ampElevatorHeightInches =
      new LoggedTunableNumber("MechanismPositions/ampElevatorHeightInches", 12.0);
  private static LoggedTunableNumber ampArmAngleDegrees =
      new LoggedTunableNumber("MechanismPositions/ampArmAngleDegrees", 50.0);

  private static LoggedTunableNumber trapElevatorHeightInches =
      new LoggedTunableNumber(
          "MechanismPositions/trapElevatorHeightInches",
          ElevatorConstants.MAX_HEIGHT.in(Units.Inches));
  private static LoggedTunableNumber trapArmAngleDegrees =
      new LoggedTunableNumber("MechanismPositions/trapArmAngleDegrees", 35.0);
  private static LoggedTunableNumber climbPrepUnderStageElevatorHeightInches =
      new LoggedTunableNumber(
          "MechanismPositions/climbPrepUnderStageElevatorHeightInches",
          Constants.ElevatorConstants.MAX_HEIGHT_BELOW_STAGE);
  private static LoggedTunableNumber climbPrepUnderStageArmAngleDegrees =
      new LoggedTunableNumber("MechanismPositions/climbPrepUnderStageArmAngleDegrees", -45.0);

  private static LoggedTunableNumber climbPrepElevatorHeightInches =
      new LoggedTunableNumber(
          "MechanismPositions/climbPrepElevatorHeightInches",
          Constants.ElevatorConstants.HEIGHT_ABOVE_CHAIN);
  private static LoggedTunableNumber climbPrepArmAngleDegrees =
      new LoggedTunableNumber("MechanismPositions/climbPrepArmAngleDegrees", 90.0);

  public record MechanismPosition(Measure<Distance> elevatorHeight, Rotation2d armAngle) {}

  public static MechanismPosition loadingPosition() {
    return new MechanismPosition(
        Units.Inches.of(homeElevatorHeightInches.get()),
        Rotation2d.fromDegrees(homeArmAngleDegrees.get()));
  }

  public static MechanismPosition ampPosition() {
    return new MechanismPosition(
        Units.Inches.of(ampElevatorHeightInches.get()),
        Rotation2d.fromDegrees(ampArmAngleDegrees.get()));
  }

  public static MechanismPosition trapPosition() {
    return new MechanismPosition(
        Units.Inches.of(trapElevatorHeightInches.get()),
        Rotation2d.fromDegrees(trapArmAngleDegrees.get()));
  }

  public static MechanismPosition climbPrepUnderStagePosition() {
    return new MechanismPosition(
        climbPrepUnderStageElevatorHeightInches.get(),
        Rotation2d.fromDegrees(climbPrepUnderStageArmAngleDegrees.get()));
  }

  public static MechanismPosition climbPrepPosition() {
    return new MechanismPosition(
        climbPrepElevatorHeightInches.get(),
        Rotation2d.fromDegrees(climbPrepArmAngleDegrees.get()));
  }
}
