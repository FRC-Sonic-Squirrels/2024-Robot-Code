package frc.robot.commands.mechanism;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.mechanism.MechanismPositions.MechanismPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.Supplier;

public class MechanismActions {

  public static Command loadingPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::loadingPosition);
  }

  public static Command ampPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::ampPosition);
  }

  private static Command goToPositionParallel(
      Elevator elevator, Arm arm, Supplier<MechanismPosition> position) {
    return new Command() {
      MechanismPosition targetPosition;

      @Override
      public void execute() {
        targetPosition = position.get();
        Measure<Distance> safeHeight = Constants.ElevatorConstants.SAFE_HEIGHT;

        if (elevator.getHeightInches() >= safeHeight.in(Units.Inches))
          arm.setAngle(targetPosition.armAngle());

        if (targetPosition.elevatorHeight().lte(safeHeight)
            && arm.getAngle().getRadians() >= Constants.ArmConstants.AMP_SAFE_ANGLE.getRadians()) {
          elevator.setHeight(safeHeight);
        } else {
          elevator.setHeight(targetPosition.elevatorHeight());
        }
      }

      @Override
      public boolean isFinished() {
        return elevator.isAtTarget(targetPosition.elevatorHeight()) && arm.isAtTargetAngle();
      }
    };
  }
}
