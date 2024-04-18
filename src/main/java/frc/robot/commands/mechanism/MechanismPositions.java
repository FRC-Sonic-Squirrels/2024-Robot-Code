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
          "home/ElevatorHeightInches",
          Constants.ElevatorConstants.LOADING_POSITION.in(Units.Inches));

  private static final LoggedTunableNumber homeArmAngleDegrees =
      group.build("home/ArmAngleDegrees", -92.0);

  private static final LoggedTunableNumber ampFastElevator = group.build("amp/Fast/Elevator", 14.0);
  private static final LoggedTunableNumber ampFastArm = group.build("amp/Fast/Arm", 50.0);

  private static final LoggedTunableNumber ampElevatorHeightInches =
      group.build("amp/Score/ElevatorHeightInches", 11.6); // 17.0

  private static final LoggedTunableNumber ampArmAngleDegrees =
      group.build("amp/Score/ArmAngleDegrees", 43); // 35.0

  private static final LoggedTunableNumber ampPrepElevatorHeightInches =
      group.build("amp/Prep/ElevatorHeightInches", 11.6);

  private static final LoggedTunableNumber ampPrepArmAngleDegrees =
      group.build("amp/Prep/ArmAngleDegrees", 70);

  private static final LoggedTunableNumber ampStage2ElevatorHeightInches =
      group.build("amp/Stage2/ElevatorHeightInches", 24.0);

  private static final LoggedTunableNumber ampStage2ArmAngleDegrees =
      group.build("amp/Stage2/ArmAngleDegrees", 0.0);

  public static final LoggedTunableNumber ampStage3ElevatorHeightInches =
      group.build("amp/Stage3/ElevatorHeightInches", 24.0);

  private static final LoggedTunableNumber ampStage3ArmAngleDegrees =
      group.build("amp/Stage3/ArmAngleDegrees", -90.0);

  private static final LoggedTunableNumber climbPrepUnderStageElevatorHeightInches =
      group.build(
          "climb/PrepUnderStage/ElevatorHeightInches",
          Constants.ElevatorConstants.MAX_HEIGHT_BELOW_STAGE.in(Units.Inches));

  private static final LoggedTunableNumber climbPrepUnderStageArmAngleDegrees =
      group.build("climb/PrepUnderStage/ArmAngleDegrees", -45.0);

  private static final LoggedTunableNumber climbPrepElevatorHeightInches =
      group.build(
          "climb/Prep/ElevatorHeightInches",
          Constants.ElevatorConstants.HEIGHT_ABOVE_CHAIN.in(Units.Inches));

  private static final LoggedTunableNumber climbPrepArmAngleDegrees =
      group.build("climb/Prep/ArmAngleDegrees", Constants.ArmConstants.MAX_ARM_ANGLE.getDegrees());

  private static final LoggedTunableNumber climbDownElevatorHeightInches =
      group.build("climb/Down/ElevatorHeightInches", 0.0);

  private static final LoggedTunableNumber climbDownArmAngleDegrees =
      group.build("climb/Down/ArmAngleDegrees", 90.0);

  private static final LoggedTunableNumber climbTrapElevatorHeightInches =
      group.build("climb/Trap/Stage1/ElevatorHeightInches", 17.5);

  private static final LoggedTunableNumber climbTrapArmAngleDegrees =
      group.build("climb/Trap/Stage1/ArmAngleDegrees", 69.0);

  private static final LoggedTunableNumber climbTrapStage2ElevatorHeightInches =
      group.build("climb/Trap/Stage2/ElevatorHeightInches", 21.575);

  private static final LoggedTunableNumber climbTrapStage2ArmAngleDegrees =
      group.build("climb/Trap/Stage2/ArmAngleDegrees", 45.0);

  private static final LoggedTunableNumber climbChainCheckElevatorHeightInches =
      group.build("climb/ChainCheck/ElevatorHeightInches", 20);

  private static final LoggedTunableNumber climbChainCheckArmAngleDegrees =
      group.build(
          "climb/ChainCheck/ArmAngleDegrees", Constants.ArmConstants.MAX_ARM_ANGLE.getDegrees());

  private static final LoggedTunableNumber climbFinalRestPositionElevatorHeightInches =
      group.build("climb/FinalRestPosition/Stage1/ElevatorHeightInches", 18);

  private static final LoggedTunableNumber climbFinalRestPositionArmAngleDegrees =
      group.build("climb/FinalRestPosition/Stage1/ArmAngleDegrees", 64.0);

  private static final LoggedTunableNumber climbFinalRestPositionStage2ElevatorHeightInches =
      group.build("climb/FinalRestPosition/Stage2/ElevatorHeightInches", 14);

  private static final LoggedTunableNumber climbFinalRestPositionStage2ArmAngleDegrees =
      group.build("climb/FinalRestPosition/Stage2/ArmAngleDegrees", 50.0);

  private static final LoggedTunableNumber climbFinalRestPositionStage3ElevatorHeightInches =
      group.build("climb/FinalRestPosition/Stage3/ElevatorHeightInches", 10);

  private static final LoggedTunableNumber climbFinalRestPositionStage3ArmAngleDegrees =
      group.build("climb/FinalRestPosition/Stage3/ArmAngleDegrees", 30.0);

  private static final LoggedTunableNumber deployReactionArms1ElevatorHeightInches =
      group.build("deployReactionArms/stage1/ElevatorHeightInches", 14.13);

  private static final LoggedTunableNumber deployReactionArms1ArmAngleDegrees =
      group.build("deployReactionArms/stage1/ArmAngleDegrees", -90);

  private static final LoggedTunableNumber deployReactionArms2ElevatorHeightInches =
      group.build("deployReactionArms/stage2/ElevatorHeightInches", 14.13);

  private static final LoggedTunableNumber deployReactionArms2ArmAngleDegrees =
      group.build("deployReactionArms/stage2/ArmAngleDegrees", -59);

  private static final LoggedTunableNumber deployReactionArms3ElevatorHeightInches =
      group.build("deployReactionArms/stage3/ElevatorHeightInches", 10.9);

  private static final LoggedTunableNumber deployReactionArms3ArmAngleDegrees =
      group.build("deployReactionArms/stage3/ArmAngleDegrees", -59);

  private static final LoggedTunableNumber deployReactionArms4ElevatorHeightInches =
      group.build("deployReactionArms/stage4/ElevatorHeightInches", 11.0);

  private static final LoggedTunableNumber deployReactionArms4ArmAngleDegrees =
      group.build("deployReactionArms/stage4/ArmAngleDegrees", -46);

  private static final LoggedTunableNumber noteGetOutElevatorHeight =
      group.build("noteGetOut/ElevatorHeight", 13.3);

  private static final LoggedTunableNumber noteGetOutArmAngle =
      group.build("noteGetOut/ArmAngle", Constants.ArmConstants.MAX_ARM_ANGLE.getDegrees());

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

  public static MechanismPosition climbTrapStage2Position() {
    return new MechanismPosition(
        Units.Inches.of(climbTrapStage2ElevatorHeightInches.get()),
        Rotation2d.fromDegrees(climbTrapStage2ArmAngleDegrees.get()));
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

  public static MechanismPosition climbFinalRestPositionStage2() {
    return new MechanismPosition(
        Units.Inches.of(climbFinalRestPositionStage2ElevatorHeightInches.get()),
        Rotation2d.fromDegrees(climbFinalRestPositionStage2ArmAngleDegrees.get()));
  }

  public static MechanismPosition climbFinalRestPositionStage3() {
    return new MechanismPosition(
        Units.Inches.of(climbFinalRestPositionStage3ElevatorHeightInches.get()),
        Rotation2d.fromDegrees(climbFinalRestPositionStage3ArmAngleDegrees.get()));
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

  public static MechanismPosition noteGetOut() {
    return new MechanismPosition(
        Units.Inches.of(noteGetOutElevatorHeight.get()),
        Rotation2d.fromDegrees(noteGetOutArmAngle.get()));
  }
}
