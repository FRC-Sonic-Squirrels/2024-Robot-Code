package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team2930.commands.ConsumeSuppliedValue;
import frc.robot.commands.arm.ArmSetAngle;
import frc.robot.commands.elevator.ElevatorManualControl;
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
    intakeDebugLayout();
    endEffectorDebugLayout();
    elevatorDebugLayout();
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

  public void intakeDebugLayout() {
    var intakeTab = Shuffleboard.getTab("Intake_Debug");
    var intakeCommandsLayout =
        intakeTab
            .getLayout("IntakeCommands", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(2, 4)
            .withProperties(Map.of("Label position", "HIDDEN"));

    var tunableVoltage =
        intakeTab.add("tunableVoltage", 0.0).withPosition(9, 3).withSize(2, 1).getEntry();

    intakeCommandsLayout.add(
        new ConsumeSuppliedValue(
            intake, () -> tunableVoltage.getDouble(0.0), intake::setPercentOut));

    var stopCommand = Commands.runOnce(() -> intake.setPercentOut(0.0), intake);
    stopCommand.runsWhenDisabled();
    stopCommand.setName("INTAKE STOP");
    intakeCommandsLayout.add(stopCommand);
  }

  public void endEffectorDebugLayout() {
    var endEffectorTab = Shuffleboard.getTab("End_Effector_Debug");
    var endEffectorCommandsLayout =
        endEffectorTab
            .getLayout("EndEffectorCommands", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(2, 4)
            .withProperties(Map.of("Label position", "HIDDEN"));

    var tunableVoltage =
        endEffectorTab.add("tunableVoltage", 0.0).withPosition(5, 1).withSize(2, 1).getEntry();

    endEffectorCommandsLayout.add(
        new ConsumeSuppliedValue(
            endEffector, () -> tunableVoltage.getDouble(0.0), endEffector::setPercentOut));

    var stopCommand = Commands.runOnce(() -> endEffector.setPercentOut(0.0), endEffector);
    stopCommand.runsWhenDisabled();
    stopCommand.setName("END EFFECTOR STOP");
    endEffectorCommandsLayout.add(stopCommand);
  }

  public void elevatorDebugLayout() {
    var elevatorTab = Shuffleboard.getTab("Elevator_Debug");
    var elevatorCommandsLayout =
        elevatorTab
            .getLayout("ElevatorCommands", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(2, 4)
            .withProperties(Map.of("Label position", "HIDDEN"));

    var tunableVoltage =
        elevatorTab.add("tunableVoltage", 0.0).withPosition(5, 1).withSize(2, 1).getEntry();

    var tunableHeight =
        elevatorTab.add("tunableHeight", 0.0).withPosition(5, 3).withSize(2, 1).getEntry();

    // elevatorCommandsLayout.add(
    //     new ConsumeSuppliedValue(
    //         elevator, () -> tunableVoltage.getDouble(0.0), elevator::setVoltage));

    elevatorCommandsLayout.add(
        new ElevatorManualControl(elevator, () -> tunableVoltage.getDouble(0.0)));

    elevatorCommandsLayout.add(
        new ConsumeSuppliedValue(
            elevator, () -> tunableHeight.getDouble(0.0), elevator::setHeight));

    var stopCommand = Commands.runOnce(() -> elevator.setVoltage(0.0), elevator);
    stopCommand.runsWhenDisabled();
    stopCommand.setName("ELEVATOR STOP");
    elevatorCommandsLayout.add(stopCommand);
  }

  // FIXME: add the rest of the shooter debug logic
  public void shooterDebugLayout() {
    var shooterTab = Shuffleboard.getTab("Shooter_Debug");
    // FIXME: change layout position
    var shooterCommandsLayout =
        shooterTab
            .getLayout("ShooterCommands", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(2, 4)
            .withProperties(Map.of("Label position", "HIDDEN"));

    var tunableVoltage =
        shooterTab.add("tunableVoltage", 0.0).withPosition(5, 1).withSize(2, 1).getEntry();

    shooterCommandsLayout.add(
        new ConsumeSuppliedValue(
            shooter, () -> tunableVoltage.getDouble(0.0), shooter::setPercentOut));

    var stopCommand = Commands.runOnce(() -> shooter.setPercentOut(0.0), shooter);
    stopCommand.runsWhenDisabled();
    stopCommand.setName("SHOOTER STOP");
    shooterCommandsLayout.add(stopCommand);
  }
}
