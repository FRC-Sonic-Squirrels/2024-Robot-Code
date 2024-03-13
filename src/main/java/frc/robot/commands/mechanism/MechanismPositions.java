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

  private static final LoggedTunableNumber homeElevatorHeightInches =
      group.build(
          "homeElevatorHeightInches",
          Constants.ElevatorConstants.LOADING_POSITION.in(Units.Inches));

  private static final LoggedTunableNumber homeArmAngleDegrees =
      group.build("homeArmAngleDegrees", -92.0);

  private static final LoggedTunableNumber ampFastElevator = group.build("ampFastElevator", 14.0);
  private static final LoggedTunableNumber ampFastArm = group.build("ampFastArm", 50.0);

  private static final LoggedTunableNumber ampElevatorHeightInches =
      group.build("ampElevatorHeightInches", 11.6); // 17.0

  private static final LoggedTunableNumber ampArmAngleDegrees =
      group.build("ampArmAngleDegrees", 43); // 35.0

  private static final LoggedTunableNumber ampPrepElevatorHeightInches =
      group.build("ampPrepElevatorHeightInches", 11.6);

  private static final LoggedTunableNumber ampPrepArmAngleDegrees =
      group.build("ampPrepArmAngleDegrees", 70);

  private static final LoggedTunableNumber ampStage2ElevatorHeightInches =
      group.build("ampStage2ElevatorHeightInches", 24.0);

  private static final LoggedTunableNumber ampStage2ArmAngleDegrees =
      group.build("ampStage2ArmAngleDegrees", 0.0);

  public static final LoggedTunableNumber ampStage3ElevatorHeightInches =
      group.build("ampStage3ElevatorHeightInches", 24.0);

  private static final LoggedTunableNumber ampStage3ArmAngleDegrees =
      group.build("ampStage3ArmAngleDegrees", -90.0);

  private static final LoggedTunableNumber climbPrepUnderStageElevatorHeightInches =
      group.build(
          "climbPrepUnderStageElevatorHeightInches",
          Constants.ElevatorConstants.MAX_HEIGHT_BELOW_STAGE.in(Units.Inches));

  private static final LoggedTunableNumber climbPrepUnderStageArmAngleDegrees =
      group.build("climbPrepUnderStageArmAngleDegrees", -45.0);

  private static final LoggedTunableNumber climbPrepElevatorHeightInches =
      group.build(
          "climbPrepElevatorHeightInches",
          Constants.ElevatorConstants.HEIGHT_ABOVE_CHAIN.in(Units.Inches));

  private static final LoggedTunableNumber climbPrepArmAngleDegrees =
      group.build("climbPrepArmAngleDegrees", Constants.ArmConstants.MAX_ARM_ANGLE.getDegrees());

  private static final LoggedTunableNumber climbDownElevatorHeightInches =
      group.build("climbDownElevatorHeightInches", 0.0);

  private static final LoggedTunableNumber climbDownArmAngleDegrees =
      group.build("climbDownArmAngleDegrees", 90.0);

  private static final LoggedTunableNumber climbTrapElevatorHeightInches =
      group.build("climbTrapElevatorHeightInches", 26.2);

  private static final LoggedTunableNumber climbTrapArmAngleDegrees =
      group.build("climbTrapArmAngleDegrees", 23.0);

  private static final LoggedTunableNumber climbChainCheckElevatorHeightInches =
      group.build("climbChainCheckElevatorHeightInches", 24);

  private static final LoggedTunableNumber climbChainCheckArmAngleDegrees =
      group.build(
          "climbChainCheckArmAngleDegrees", Constants.ArmConstants.MAX_ARM_ANGLE.getDegrees());

  private static final LoggedTunableNumber climbFinalRestPositionElevatorHeightInches =
      group.build("climbFinalRestPositionElevatorHeightInches", 18);

  private static final LoggedTunableNumber climbFinalRestPositionArmAngleDegrees =
      group.build("climbFinalRestPositionArmAngleDegrees", 23.0);

  private static final LoggedTunableNumber deployReactionArms1ElevatorHeightInches =
      group.build("deployReactionArms1ElevatorHeightInches", 14.13);

  private static final LoggedTunableNumber deployReactionArms1ArmAngleDegrees =
      group.build("deployReactionArms1ArmAngleDegrees", -90);

  private static final LoggedTunableNumber deployReactionArms2ElevatorHeightInches =
      group.build("deployReactionArms2ElevatorHeightInches", 14.13);

  private static final LoggedTunableNumber deployReactionArms2ArmAngleDegrees =
      group.build("deployReactionArms2ArmAngleDegrees", -59);

  private static final LoggedTunableNumber deployReactionArms3ElevatorHeightInches =
      group.build("deployReactionArms3ElevatorHeightInches", 10.9);

  private static final LoggedTunableNumber deployReactionArms3ArmAngleDegrees =
      group.build("deployReactionArms3ArmAngleDegrees", -59);

  private static final LoggedTunableNumber deployReactionArms4ElevatorHeightInches =
      group.build("deployReactionArms4ElevatorHeightInches", 11.0);

  private static final LoggedTunableNumber deployReactionArms4ArmAngleDegrees =
      group.build("deployReactionArms4ArmAngleDegrees", -46);

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

  public static MechanismPosition AmpPrepPosition() {
    return new MechanismPosition(
        Units.Inches.of(ampPrepElevatorHeightInches.get()),
        Rotation2d.fromDegrees(ampPrepArmAngleDegrees.get()));
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

  //   public static MechanismPosition trapPosition() {
  //     return new MechanismPosition(
  //         Units.Inches.of(trapElevatorHeightInches.get()),
  //         Rotation2d.fromDegrees(trapArmAngleDegrees.get()));
  //   }

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

  public static MechanismPosition climbChainCheck() {
    return new MechanismPosition(
        Units.Inches.of(climbChainCheckElevatorHeightInches.get()),
        Rotation2d.fromDegrees(climbChainCheckArmAngleDegrees.get()));
  }

  public static MechanismPosition climbFinalRestPosition() {
    return new MechanismPosition(
        Units.Inches.of(climbFinalRestPositionElevatorHeightInches.get()),
        Rotation2d.fromDegrees(climbFinalRestPositionArmAngleDegrees.get()));
  }

  public static MechanismPosition deployReactionArmsStep1() {
    return new MechanismPosition(
        Units.Inches.of(deployReactionArms1ElevatorHeightInches.get()),
        Rotation2d.fromDegrees(deployReactionArms1ArmAngleDegrees.get()));
  }

  public static MechanismPosition deployReactionArmsStep2() {
    return new MechanismPosition(
        Units.Inches.of(deployReactionArms2ElevatorHeightInches.get()),
        Rotation2d.fromDegrees(deployReactionArms2ArmAngleDegrees.get()));
  }

  public static MechanismPosition deployReactionArmsStep3() {
    return new MechanismPosition(
        Units.Inches.of(deployReactionArms3ElevatorHeightInches.get()),
        Rotation2d.fromDegrees(deployReactionArms3ArmAngleDegrees.get()));
  }

  public static MechanismPosition deployReactionArmsStep4() {
    return new MechanismPosition(
        Units.Inches.of(deployReactionArms4ElevatorHeightInches.get()),
        Rotation2d.fromDegrees(deployReactionArms4ArmAngleDegrees.get()));
  }
}
