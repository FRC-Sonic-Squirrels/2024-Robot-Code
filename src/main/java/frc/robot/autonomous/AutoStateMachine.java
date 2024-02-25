// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.StateMachine;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.commands.shooter.ShooterScoreSpeakerStateMachine;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import org.littletonrobotics.junction.Logger;

public class AutoStateMachine extends StateMachine {
  private Command scoreSpeaker;
  private final DrivetrainWrapper drive;
  private final Shooter shooter;
  private final EndEffector endEffector;
  private final Elevator elevator;
  private final Arm arm;
  private final Intake intake;
  private final StateMachine[] subStates;
  private int currentSubState;
  private double initShootDeadline;

  /** Creates a new AutoSubstateMachine. */
  public AutoStateMachine(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      Elevator elevator,
      Arm arm,
      Intake intake,
      StateMachine[] subStates,
      double initShootDeadline) {
    this.drive = drive;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;
    this.subStates = subStates;
    this.initShootDeadline = initShootDeadline;

    setInitialState(this::autoInitialState);
  }

  private StateHandler autoInitialState() {
    scoreSpeaker =
        ShooterScoreSpeakerStateMachine.getAsCommand(drive, shooter, endEffector, intake, 5);
    scoreSpeaker.schedule();

    MechanismActions.loadingPosition(elevator, arm);
    Logger.recordOutput("Autonomous/CommandScheduled", true);

    return this::initialShot;
  }

  private StateHandler initialShot() {
    if (scoreSpeaker.isFinished()) {
      return this::nextSubState;
    }
    return null;
  }

  private StateHandler nextSubState() {
    if (currentSubState >= subStates.length) {
      return setDone();
    }

    return suspendForSubStateMachine(
        subStates[currentSubState++], subStateMachine -> AutoStateMachine.this::nextSubState);
  }
}
