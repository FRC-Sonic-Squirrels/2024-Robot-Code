package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.mechanism.MechanismPositions.MechanismPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.Supplier;

public class MechanismActions {

  public static Command loadingPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::loadingPosition);
  }

  public static Command ampFast(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::AmpFastPosition);
  }

  public static Command ampPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::ampPosition);
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

  public static Command climbPrepPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::climbPrepPosition);
  }

  private static Command goToPositionParallel(
      Elevator elevator, Arm arm, Supplier<MechanismPosition> position) {

    return Commands.run(
            () -> {
              MechanismPosition targetPosition = position.get();
              Measure<Distance> safeHeight = Constants.ElevatorConstants.SAFE_HEIGHT;

              if (elevator.getHeightInches()
                      >= safeHeight.minus(Units.Inches.of(1.0)).in(Units.Inches)
                  || position.get().armAngle().getRadians()
                      <= Constants.ArmConstants.ARM_SAFE_ANGLE.getRadians()) {
                arm.setAngle(targetPosition.armAngle());
              }

              if (targetPosition.elevatorHeight().lte(safeHeight)
                  && arm.getAngle().getRadians()
                      >= Constants.ArmConstants.ARM_SAFE_ANGLE.getRadians()) {
                elevator.setHeight(safeHeight);
              } else {
                elevator.setHeight(targetPosition.elevatorHeight());
              }
            },
            elevator,
            arm)
        .until(
            () ->
                elevator.isAtTarget(position.get().elevatorHeight())
                    && arm.isAtTargetAngle(position.get().armAngle(), Rotation2d.fromDegrees(5.0)));
  }
}
