package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team2930.commands.ConsumeSuppliedValue;
import frc.robot.commands.arm.ArmSetAngle;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Drivetrain;
import java.util.Map;

public class ShuffleBoardLayouts {
  Arm arm;
  Elevator elevator;
  EndEffector endEffector;
  Intake intake;
  Shooter shooter;
  Drivetrain drivetrain;

  public ShuffleBoardLayouts(
      Arm arm,
      Elevator elevator,
      EndEffector endEffector,
      Intake intake,
      Shooter shooter,
      Drivetrain drivetrain) {
    this.arm = arm;
    this.elevator = elevator;
    this.endEffector = endEffector;
    this.intake = intake;
    this.shooter = shooter;
    this.drivetrain = drivetrain;

    if (DriverStation.isFMSAttached()) {
      loadAllCompetitionLayouts();
    } else {
      loadAllNonFMSLayouts();
    }
  }

  private void loadAllCompetitionLayouts() {}

  private void loadAllNonFMSLayouts() {
    armDebugLayout();
  }

  private void armDebugLayout() {
    var armTab = Shuffleboard.getTab("Arm_Debug");
    var armCommandsLayout =
        armTab
            .getLayout("ArmCommands", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(2, 4)
            .withProperties(Map.of("Label position", "HIDDEN"));

    var tuneableAngleSetpoint =
        armTab.add("tunableSetpointDegrees", 0.0).withPosition(9, 0).withSize(2, 1).getEntry();
    var tunableVoltage =
        armTab.add("tunableVoltage", 0.0).withPosition(9, 1).withSize(2, 1).getEntry();

    armCommandsLayout.add(
        new ArmSetAngle(arm, () -> Rotation2d.fromDegrees(tuneableAngleSetpoint.getDouble(0.0))));
    armCommandsLayout.add(
        new ConsumeSuppliedValue(arm, () -> tunableVoltage.getDouble(0.0), arm::setVoltage));

    var stopCommand = Commands.runOnce(() -> arm.setVoltage(0.0), arm);
    stopCommand.runsWhenDisabled();
    stopCommand.setName("ARM STOP");
    armCommandsLayout.add(stopCommand);

    var resetSensorPositionHome = Commands.runOnce(() -> arm.resetSensorToHomePosition(), arm);
    resetSensorPositionHome.runsWhenDisabled();
    resetSensorPositionHome.setName("resetSensorToHomePosition");
    armCommandsLayout.add(resetSensorPositionHome);
  }
}
