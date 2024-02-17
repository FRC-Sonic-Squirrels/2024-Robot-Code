package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.MechanismPositions.MechanismPosition;
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
        if (elevator.getHeightInches() >= Constants.ElevatorConstants.SAFE_HEIGHT_INCHES)
          arm.setAngle(targetPosition.armAngle());

        if (targetPosition.elevatorHeight() <= Constants.ElevatorConstants.SAFE_HEIGHT_INCHES
            && arm.getAngle().getRadians() >= Constants.ArmConstants.AMP_SAFE_ANGLE.getRadians()) {
          elevator.setHeight(Constants.ElevatorConstants.SAFE_HEIGHT_INCHES);
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
