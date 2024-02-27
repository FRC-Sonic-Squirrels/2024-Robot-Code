package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;

public class MechanismPositions {
  private static final TunableNumberGroup group = new TunableNumberGroup("MechanismPositions");

  private static LoggedTunableNumber homeElevatorHeightInches =
      group.build(
          "homeElevatorHeightInches",
          Constants.ElevatorConstants.LOADING_POSITION.in(Units.Inches));
  private static LoggedTunableNumber homeArmAngleDegrees =
      group.build("homeArmAngleDegrees", -92.0);

  private static LoggedTunableNumber ampFastElevator = group.build("ampFastElevator", 12.0);
  private static LoggedTunableNumber ampFastArm = group.build("ampFastArm", 50.0);

  private static LoggedTunableNumber ampElevatorHeightInches =
      group.build("ampElevatorHeightInches", 17.0);
  private static LoggedTunableNumber ampArmAngleDegrees = group.build("ampArmAngleDegrees", 35.0);

  private static LoggedTunableNumber ampStage2ElevatorHeightInches =
      group.build("ampStage2ElevatorHeightInches", 24.0);
  private static LoggedTunableNumber ampStage2ArmAngleDegrees =
      group.build("ampStage2ArmAngleDegrees", 0.0);

  private static LoggedTunableNumber ampStage3ElevatorHeightInches =
      group.build("ampStage3ElevatorHeightInches", 24.0);
  private static LoggedTunableNumber ampStage3ArmAngleDegrees =
      group.build("ampStage3ArmAngleDegrees", -90.0);

  private static LoggedTunableNumber trapElevatorHeightInches =
      group.build("trapElevatorHeightInches", 26.2);
  private static LoggedTunableNumber trapArmAngleDegrees = group.build("trapArmAngleDegrees", 23.0);
  private static LoggedTunableNumber climbPrepUnderStageElevatorHeightInches =
      group.build(
          "climbPrepUnderStageElevatorHeightInches",
          Constants.ElevatorConstants.MAX_HEIGHT_BELOW_STAGE.in(Units.Inches));
  private static LoggedTunableNumber climbPrepUnderStageArmAngleDegrees =
      group.build("climbPrepUnderStageArmAngleDegrees", -45.0);

  private static LoggedTunableNumber climbPrepElevatorHeightInches =
      group.build(
          "climbPrepElevatorHeightInches",
          Constants.ElevatorConstants.HEIGHT_ABOVE_CHAIN.in(Units.Inches));
  private static LoggedTunableNumber climbPrepArmAngleDegrees =
      group.build("climbPrepArmAngleDegrees", Constants.ArmConstants.MAX_ARM_ANGLE.getDegrees());

  private static LoggedTunableNumber climbDownElevatorHeightInches =
      group.build("climbDownElevatorHeightInches", 0.0);
  private static LoggedTunableNumber climbDownArmAngleDegrees =
      group.build("climbDownArmAngleDegrees", 90.0);

  private static LoggedTunableNumber climbTrapElevatorHeightInches =
      group.build(
          "climbTrapElevatorHeightInches", Constants.ElevatorConstants.MAX_HEIGHT.in(Units.Inches));
  private static LoggedTunableNumber climbTrapArmAngleDegrees =
      group.build("climbTrapArmAngleDegrees", 45.0);

  private static LoggedTunableNumber climbTrapPushElevatorHeightInches =
      group.build(
          "climbTrapPushElevatorHeightInches",
          Constants.ElevatorConstants.MAX_HEIGHT.in(Units.Inches));
  private static LoggedTunableNumber climbTrapPushArmAngleDegrees =
      group.build("climbTrapPushArmAngleDegrees", 30.0);

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

  public static MechanismPosition climbDownPosition() {
    return new MechanismPosition(
        Units.Inches.of(climbDownElevatorHeightInches.get()),
        Rotation2d.fromDegrees(climbDownArmAngleDegrees.get()));
  }

  public static MechanismPosition climbTrapPosition() {
    return new MechanismPosition(
        Units.Inches.of(climbTrapElevatorHeightInches.get()),
        Rotation2d.fromDegrees(climbTrapArmAngleDegrees.get()));
  }

  public static MechanismPosition climbTrapPushPosition() {
    return new MechanismPosition(
        Units.Inches.of(climbTrapPushElevatorHeightInches.get()),
        Rotation2d.fromDegrees(climbTrapPushArmAngleDegrees.get()));
  }
}
