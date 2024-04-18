package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.commands.endEffector.EndEffectorCenterNoteBetweenToFs;
import frc.robot.commands.endEffector.EndEffectorPrepareNoteForTrap;
import frc.robot.commands.mechanism.MechanismPositions.MechanismPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class MechanismActions {
  private static final String ROOT_TABLE = "MechanismActions";

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Bool log_runningArm = logGroup.buildBoolean("runningArm");
  private static final LoggerEntry.Bool log_runningElevator =
      logGroup.buildBoolean("runningElevator");
  private static final LoggerEntry.Bool log_ElevatorInPosition =
      logGroup.buildBoolean("ElevatorInPosition");
  private static final LoggerEntry.Bool log_ArmInPosition = logGroup.buildBoolean("ArmInPosition");

  private static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);

  public static final LoggedTunableNumber climbDownElevatorVelocity =
      group.build("MechanismActions/climbDownElevatorVelocity", 500);
  public static final LoggedTunableNumber climbDownElevatorAcceleration =
      group.build("climbDownElevatorAcceleration", 500);

  public static final LoggedTunableNumber tunableArmVoltage =
      group.build("MechanismActions/tunableArmVoltage", 5.0);

  public static Command loadingPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::loadingPosition);
  }

  public static Command ampFast(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::AmpFastPosition);
  }

  public static Command ampPrepPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::AmpPrepPosition);
  }

  public static Command ampPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::ampPosition);
  }

  // public static Command ampPosition(Elevator elevator, Arm arm) {
  // return ampStage3Position(elevator, arm)
  // .andThen(ampStage2Position(elevator, arm))
  // .andThen(ampStage1Position(elevator, arm));
  // }

  public static Command ampPositionToLoadPosition(Elevator elevator, Arm arm) {
    return ampStage1Position(elevator, arm)
        .andThen(ampStage2Position(elevator, arm))
        .andThen(ampStage3Position(elevator, arm))
        .andThen(loadingPosition(elevator, arm));
  }

  public static Command ampStage1Position(Elevator elevator, Arm arm) {
    return goToPositionParallel(
        elevator,
        arm,
        () -> new MechanismPosition(Units.Inches.of(24.0), Rotation2d.fromDegrees(0.0)));
  }

  public static Command ampStage2Position(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::ampStage2Position);
  }

  public static Command ampStage3Position(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::ampStage3Position);
  }

  public static Command climbPrepUnderStagePosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::climbPrepUnderStagePosition);
  }

  public static Command climbPrepPosition(
      Elevator elevator, Arm arm, EndEffector endEffector, Shooter shooter, Intake intake) {
    return new ConditionalCommand(
            goToPositionParallel(
                elevator,
                arm,
                () ->
                    new MechanismPosition(Units.Inch.of(14), Constants.ArmConstants.MAX_ARM_ANGLE)),
            Commands.none(),
            () -> elevator.getHeightInches() < 14)
        .andThen(goToPositionParallel(elevator, arm, MechanismPositions::climbPrepPosition))
        .deadlineWith(new EndEffectorCenterNoteBetweenToFs(endEffector, intake, shooter))
        .andThen(new EndEffectorPrepareNoteForTrap(endEffector));
  }

  public static Command climbDownPosition(Elevator elevator, Arm arm) {
    return goToPositionParallelSetMotionConstraints(
            elevator,
            arm,
            () -> new MechanismPosition(Units.Inches.of(16), Constants.ArmConstants.MAX_ARM_ANGLE),
            () -> climbDownElevatorVelocity.get(),
            () -> climbDownElevatorAcceleration.get())
        .andThen(
            goToPositionParallelSetMotionConstraints(
                elevator,
                arm,
                MechanismPositions::climbDownPosition,
                () -> climbDownElevatorVelocity.get(),
                () -> climbDownElevatorAcceleration.get()));
  }

  public static Command climbTrapPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::climbTrapPosition)
        .andThen(goToPositionParallel(elevator, arm, MechanismPositions::climbTrapStage2Position));
  }

  public static Command climbChainCheck(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::climbChainCheck);
  }

  public static Command climbFinalRestPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::climbFinalRestPosition)
        .andThen(
            goToPositionParallel(elevator, arm, MechanismPositions::climbFinalRestPositionStage2))
        .andThen(
            goToPositionParallel(elevator, arm, MechanismPositions::climbFinalRestPositionStage3));
  }

  public static Command climbFinalRestPositionStage2(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::climbFinalRestPositionStage2);
  }

  public static Command climbFinalRestPositionArmVoltage(Elevator elevator, Arm arm) {
    return Commands.run(() -> arm.setVoltage(tunableArmVoltage.get()), arm)
        .alongWith(
            Commands.runOnce(
                () ->
                    elevator.setHeight(
                        MechanismPositions.climbFinalRestPositionStage2().elevatorHeight()),
                elevator))
        .andThen(Commands.waitUntil(elevator::isAtTarget))
        .andThen(Commands.runOnce(() -> arm.setVoltage(0.0), arm));
  }

  public static Command deployReactionArms(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::deployReactionArmsStep1, true)
        .andThen(
            goToPositionParallel(elevator, arm, MechanismPositions::deployReactionArmsStep2, true))
        .andThen(
            goToPositionParallel(elevator, arm, MechanismPositions::deployReactionArmsStep3, true))
        .andThen(
            goToPositionParallel(elevator, arm, MechanismPositions::deployReactionArmsStep4, true));
  }

  public static Command noteGetOut(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::noteGetOut);
  }

  private static Command goToPositionParallel(
      Elevator elevator, Arm arm, Supplier<MechanismPosition> position) {
    return goToPositionParallel(elevator, arm, position, false);
  }

  private static Command goToPositionParallel(
      Elevator elevator, Arm arm, Supplier<MechanismPosition> position, boolean ignoreSafety) {

    var cmd =
        new Command() {
          boolean elevatorInPosition = false;
          boolean armInPosition = false;

          @Override
          public void execute() {
            MechanismPosition targetPosition = position.get();
            Measure<Distance> safeHeight = Constants.ElevatorConstants.SAFE_HEIGHT;
            boolean runningArm =
                elevator.getHeightInches()
                        >= safeHeight.minus(Units.Inches.of(1.0)).in(Units.Inches)
                    || (arm.getAngle().getRadians()
                            > Constants.ArmConstants.ARM_SAFE_ANGLE.getRadians()
                        && position.get().armAngle().getRadians()
                            > Constants.ArmConstants.ARM_SAFE_ANGLE.getRadians());
            if (ignoreSafety) runningArm = true;
            if (runningArm) {
              arm.setAngle(targetPosition.armAngle());
            }

            boolean runningElevatorSafety =
                targetPosition.elevatorHeight().lte(safeHeight)
                    && ((arm.getAngle().getRadians()
                                >= Constants.ArmConstants.ARM_SAFE_ANGLE.getRadians()
                            && targetPosition.armAngle().getRadians()
                                <= Constants.ArmConstants.ARM_SAFE_ANGLE.getRadians())
                        || (arm.getAngle().getRadians()
                                <= Constants.ArmConstants.ARM_SAFE_ANGLE.getRadians()
                            && position.get().armAngle().getRadians()
                                >= Constants.ArmConstants.ARM_SAFE_ANGLE.getRadians()));
            if (ignoreSafety) runningElevatorSafety = false;
            // && targetPosition.armAngle().getRadians() >= Math.toRadians(60.0)

            if (runningElevatorSafety) {
              elevator.setHeight(safeHeight);
            } else {
              elevator.setHeight(targetPosition.elevatorHeight());
            }
            log_runningArm.info(runningArm);
            log_runningElevator.info(runningElevatorSafety);
            elevatorInPosition = elevator.isAtTarget(position.get().elevatorHeight());
            log_ElevatorInPosition.info(elevatorInPosition);
            armInPosition =
                arm.isAtTargetAngle(position.get().armAngle(), Rotation2d.fromDegrees(5.0));
            log_ArmInPosition.info(armInPosition);
          }

          @Override
          public boolean isFinished() {
            // TODO Auto-generated method stub
            return elevatorInPosition && armInPosition;
          }
        };

    cmd.addRequirements(elevator, arm);
    cmd.setName("MechanismAction");
    return cmd;
  }

  private static Command goToPositionParallelSetMotionConstraints(
      Elevator elevator,
      Arm arm,
      Supplier<MechanismPosition> position,
      DoubleSupplier maxElevatorVelocity,
      DoubleSupplier maxElevatorAcceleration) {
    return Commands.sequence(
        setElevatorMotionMagicCommand(elevator, maxElevatorVelocity, maxElevatorAcceleration),
        goToPositionParallel(elevator, arm, position),
        setElevatorMotionMagicCommand(
            elevator,
            () -> elevator.getDefaultMotionMagicConstraints().maxVelocity,
            () -> elevator.getDefaultMotionMagicConstraints().maxAcceleration));
  }

  private static Command setElevatorMotionMagicCommand(
      Elevator elevator,
      DoubleSupplier maxElevatorVelocity,
      DoubleSupplier maxElevatorAcceleration) {
    return Commands.runOnce(
        () -> {
          double vel = maxElevatorVelocity.getAsDouble();
          double acc = maxElevatorAcceleration.getAsDouble();

          if (vel != elevator.getCurrentMotionMagicConstraints().maxVelocity
              || acc != elevator.getCurrentMotionMagicConstraints().maxAcceleration) {

            elevator.setMotionMagicConstraints(
                new Constraints(
                    maxElevatorVelocity.getAsDouble(), maxElevatorAcceleration.getAsDouble()));
          }
        },
        elevator);
  }
}
