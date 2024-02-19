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
    shooterDebugLayout();
    systemsCheck();
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
        armTab.add("tunableSetpointDegrees", 0.0).withPosition(2, 0).withSize(2, 1).getEntry();
    var tunableVoltage =
        armTab.add("tunableVoltage", 0.0).withPosition(2, 1).withSize(2, 1).getEntry();

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
        intakeTab.add("tunableVoltage", 0.0).withPosition(2, 0).withSize(2, 1).getEntry();

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
        endEffectorTab.add("tunableVoltage", 0.0).withPosition(2, 0).withSize(2, 1).getEntry();

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
        elevatorTab.add("tunableVoltage", 0.0).withPosition(2, 0).withSize(2, 1).getEntry();

    var tunableHeight =
        elevatorTab.add("tunableHeight", 0.0).withPosition(2, 1).withSize(2, 1).getEntry();

    var setVoltage = new ElevatorManualControl(elevator, () -> tunableVoltage.getDouble(0.0));
    setVoltage.setName("setVoltage");
    elevatorCommandsLayout.add(setVoltage);

    var setHeight = new ConsumeSuppliedValue(elevator, () -> tunableHeight.getDouble(0.0), elevator::setHeight);
    setHeight.setName("setHeight");
    elevatorCommandsLayout.add(setHeight);

    var stopCommand =
        Commands.sequence(
            Commands.runOnce(() -> elevator.setHeight(0.0), elevator),
            Commands.runOnce(() -> elevator.setVoltage(0.0), elevator));
    stopCommand.runsWhenDisabled();
    stopCommand.setName("ELEVATOR STOP");
    elevatorCommandsLayout.add(stopCommand);
  }

  public void shooterDebugLayout() {
    var shooterTab = Shuffleboard.getTab("Shooter_Debug");
    var shooterCommandsLayout =
        shooterTab
            .getLayout("ShooterCommands", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(2, 4)
            .withProperties(Map.of("Label position", "HIDDEN"));

    var tunablePercentOut =
        shooterTab.add("tunablePercentOut", 0.0).withPosition(2, 0).withSize(2, 1).getEntry();

    var tunablePivotDegrees =
        shooterTab.add("tunablePivotDegrees", 0.0).withPosition(2, 1).withSize(2, 1).getEntry();

    var tunableRPM = shooterTab.add("tunableRPM", 0.0).withPosition(2, 2).withSize(2, 1).getEntry();

    // MOTORS

    var setKickerPercentOut =
        new ConsumeSuppliedValue(
                shooter, () -> tunablePercentOut.getDouble(0.0), shooter::setKickerPercentOut)
            .finallyDo(() -> shooter.setKickerPercentOut(0.0));
    setKickerPercentOut.setName("setKickerPercentOut");

    var setLeadPercentOut =
        new ConsumeSuppliedValue(
                shooter, () -> tunablePercentOut.getDouble(0.0), shooter::setPercentOut)
            .finallyDo(() -> shooter.setPercentOut(0.0));
    setLeadPercentOut.setName("setLeadPercentOut");

    var setLauncherRPM =
        new ConsumeSuppliedValue(shooter, () -> tunableRPM.getDouble(0.0), shooter::setLauncherRPM)
            .finallyDo(() -> shooter.setLauncherRPM(0.0));
    setLauncherRPM.setName("setLauncherRPM");

    var setPivotPosition =
        Commands.runOnce(
            () ->
                shooter.setPivotPosition(
                    Rotation2d.fromDegrees(tunablePivotDegrees.getDouble(0.0))),
            shooter);
    setPivotPosition.setName("setPivotPosition");

    shooterCommandsLayout.add(setKickerPercentOut);
    shooterCommandsLayout.add(setLeadPercentOut);
    shooterCommandsLayout.add(setLauncherRPM);
    shooterCommandsLayout.add(setPivotPosition);

    var stopCommand =
        Commands.sequence(
            Commands.runOnce(() -> shooter.setPivotPosition(Rotation2d.fromDegrees(0)), shooter),
            Commands.runOnce(() -> shooter.setPivotVoltage(0.0), shooter),
            Commands.runOnce(() -> shooter.setLauncherVoltage(0.0), shooter));

    stopCommand.runsWhenDisabled();
    stopCommand.setName("SHOOTER STOP");
    shooterCommandsLayout.add(stopCommand);
  }

  // FIXME: add drive and turn commands
  public void systemsCheck() {
    var systemsCheckTab = Shuffleboard.getTab("Systems_Check");
    var systemsCheckCommandsLayout =
        systemsCheckTab
            .getLayout("SystemsChecks", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(2, 4)
            .withProperties(Map.of("Label position", "HIDDEN"));

    // TODO: add command that replaces elevator and arm check
    // var checkArmAndElevator;
    
    // checks the elevator height
    var checkElevator =
        Commands.sequence(
            Commands.runOnce(
                () -> elevator.setHeight(Constants.ElevatorConstants.MAX_HEIGHT), elevator),
            Commands.waitUntil(() -> elevator.isAtTarget()),
            Commands.runOnce(() -> elevator.setHeight(0.0), elevator));

    // checks the arm angle
    var checkArm =
        Commands.sequence(
            Commands.runOnce(() -> arm.setAngle(Constants.ArmConstants.MAX_ARM_ANGLE), arm),
            Commands.waitUntil(() -> arm.getAngle() == Constants.ArmConstants.MAX_ARM_ANGLE),
            Commands.runOnce(() -> arm.setAngle(Rotation2d.fromDegrees(0.0)), arm));

    // checks percent output of end effector motor
    var checkEndEffector =
        Commands.sequence(
            Commands.runOnce(
                () -> endEffector.setPercentOut(Constants.EndEffectorConstants.INDEX_PERCENT_OUT),
                endEffector),
            Commands.waitSeconds(3),
            Commands.runOnce(() -> endEffector.setPercentOut(0.0), endEffector));

    // checks percent output of intake motor
    var checkIntake =
        Commands.sequence(
            Commands.runOnce(
                () -> intake.setPercentOut(Constants.IntakeConstants.INTAKE_IDLE_PERCENT_OUT),
                intake),
            Commands.waitSeconds(3),
            Commands.runOnce(() -> intake.setPercentOut(0.0), intake));

    // checks percent out of kicker and launcher motors, and pivot angle
    var checkShooter =
        Commands.sequence(
            Commands.runOnce(
                () ->
                    shooter.setKickerPercentOut(
                        Constants.ShooterConstants.Kicker.KICKING_PERCENT_OUT),
                shooter),
            Commands.waitSeconds(3),
            Commands.runOnce(() -> shooter.setKickerPercentOut(0.0), shooter),
            Commands.runOnce(
                () -> shooter.setPercentOut(Constants.ShooterConstants.SHOOTING_PERCENT_OUT),
                shooter),
            Commands.waitSeconds(3),
            Commands.runOnce(() -> shooter.setPercentOut(0.0), shooter),
            Commands.runOnce(
                () -> shooter.setPivotPosition(Constants.ShooterConstants.Pivot.MAX_ANGLE_RAD),
                shooter),
            Commands.waitUntil(
                () -> shooter.getPitch() == Constants.ShooterConstants.Pivot.MAX_ANGLE_RAD),
            Commands.runOnce(() -> shooter.setPivotPosition(Rotation2d.fromDegrees(0.0)), shooter));

    checkElevator.setName("Check Elevator Height");
    checkArm.setName("Check Arm Angle");
    checkEndEffector.setName("Check End Effector");
    checkIntake.setName("Check Intake");
    checkShooter.setName("Check Shooter");

    systemsCheckCommandsLayout.add(checkElevator);
    systemsCheckCommandsLayout.add(checkArm);
    systemsCheckCommandsLayout.add(checkEndEffector);
    systemsCheckCommandsLayout.add(checkIntake);
    systemsCheckCommandsLayout.add(checkShooter);
  }
}
