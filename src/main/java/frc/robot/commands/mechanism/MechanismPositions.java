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
      new LoggedTunableNumber(
          "MechanismPositions/homeElevatorHeightInches",
          Constants.ElevatorConstants.LOADING_POSITION.in(Units.Inches));
  private static LoggedTunableNumber homeArmAngleDegrees =
      new LoggedTunableNumber("MechanismPositions/homeArmAngleDegrees", -92.0);

  private static LoggedTunableNumber ampFastElevator =
      new LoggedTunableNumber("MechanismPositions/ampFastElevator", 12.0);
  private static LoggedTunableNumber ampFastArm =
      new LoggedTunableNumber("MechanismPositions/ampFastArm", 50.0);

  private static LoggedTunableNumber ampElevatorHeightInches =
      new LoggedTunableNumber("MechanismPositions/ampElevatorHeightInches", 17.0);
  private static LoggedTunableNumber ampArmAngleDegrees =
      new LoggedTunableNumber("MechanismPositions/ampArmAngleDegrees", 35.0);

  private static LoggedTunableNumber ampStage2ElevatorHeightInches =
      new LoggedTunableNumber("MechanismPositions/ampStage2ElevatorHeightInches", 24.0);
  private static LoggedTunableNumber ampStage2ArmAngleDegrees =
      new LoggedTunableNumber("MechanismPositions/ampStage2ArmAngleDegrees", 0.0);

  private static LoggedTunableNumber ampStage3ElevatorHeightInches =
      new LoggedTunableNumber("MechanismPositions/ampStage3ElevatorHeightInches", 24.0);
  private static LoggedTunableNumber ampStage3ArmAngleDegrees =
      new LoggedTunableNumber("MechanismPositions/ampStage3ArmAngleDegrees", -90.0);

  private static LoggedTunableNumber trapElevatorHeightInches =
      new LoggedTunableNumber(
          "MechanismPositions/trapElevatorHeightInches",
          ElevatorConstants.MAX_HEIGHT.in(Units.Inches));
  private static LoggedTunableNumber trapArmAngleDegrees =
      new LoggedTunableNumber("MechanismPositions/trapArmAngleDegrees", 35.0);
  private static LoggedTunableNumber climbPrepUnderStageElevatorHeightInches =
      new LoggedTunableNumber(
          "MechanismPositions/climbPrepUnderStageElevatorHeightInches",
          Constants.ElevatorConstants.MAX_HEIGHT_BELOW_STAGE.in(Units.Inches));
  private static LoggedTunableNumber climbPrepUnderStageArmAngleDegrees =
      new LoggedTunableNumber("MechanismPositions/climbPrepUnderStageArmAngleDegrees", -45.0);

  private static LoggedTunableNumber climbPrepElevatorHeightInches =
      new LoggedTunableNumber(
          "MechanismPositions/climbPrepElevatorHeightInches",
          Constants.ElevatorConstants.HEIGHT_ABOVE_CHAIN.in(Units.Inches));
  private static LoggedTunableNumber climbPrepArmAngleDegrees =
      new LoggedTunableNumber(
          "MechanismPositions/climbPrepArmAngleDegrees",
          Constants.ArmConstants.MAX_ARM_ANGLE.getDegrees());

  public record MechanismPosition(Measure<Distance> elevatorHeight, Rotation2d armAngle) {}

  public static MechanismPosition loadingPosition() {
    return new MechanismPosition(
        Units.Inches.of(homeElevatorHeightInches.get()),
        Rotation2d.fromDegrees(homeArmAngleDegrees.get()));
  }

  public static MechanismPosition AmpFastPosition() {
    return new MechanismPosition(
        Units.Inches.of(ampFastElevator.get()), Rotation2d.fromDegrees(ampFastArm.get()));
  }

  public static MechanismPosition ampPosition() {
    return new MechanismPosition(
        Units.Inches.of(ampElevatorHeightInches.get()),
        Rotation2d.fromDegrees(ampArmAngleDegrees.get()));
  }

  public static MechanismPosition ampStage2Position() {
    return new MechanismPosition(
        Units.Inches.of(ampStage2ElevatorHeightInches.get()),
        Rotation2d.fromDegrees(ampStage2ArmAngleDegrees.get()));
  }

  public static MechanismPosition ampStage3Position() {
    return new MechanismPosition(
        Units.Inches.of(ampStage3ElevatorHeightInches.get()),
        Rotation2d.fromDegrees(ampStage3ArmAngleDegrees.get()));
  }

  public static MechanismPosition trapPosition() {
    return new MechanismPosition(
        Units.Inches.of(trapElevatorHeightInches.get()),
        Rotation2d.fromDegrees(trapArmAngleDegrees.get()));
  }

  public static MechanismPosition climbPrepUnderStagePosition() {
    return new MechanismPosition(
        Units.Inches.of(climbPrepUnderStageElevatorHeightInches.get()),
        Rotation2d.fromDegrees(climbPrepUnderStageArmAngleDegrees.get()));
  }

  public static MechanismPosition climbPrepPosition() {
    return new MechanismPosition(
        Units.Inches.of(climbPrepElevatorHeightInches.get()),
        Rotation2d.fromDegrees(climbPrepArmAngleDegrees.get()));
  }
}
