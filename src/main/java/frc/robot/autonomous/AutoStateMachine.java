// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import frc.lib.team2930.StateMachine;
import frc.robot.DrivetrainWrapper;
import frc.robot.commands.ScoreSpeaker;
import frc.robot.subsystems.shooter.Shooter;

public class AutoStateMachine extends StateMachine {
  private final DrivetrainWrapper drive;
  private final Shooter shooter;
  private final AutoSubstateMachine[] subStates;
  private int currentSubState;

  /** Creates a new AutoSubstateMachine. */
  public AutoStateMachine(
      DrivetrainWrapper drive, Shooter shooter, AutoSubstateMachine[] subStates) {
    this.drive = drive;
    this.shooter = shooter;
    this.subStates = subStates;

    setInitialState(makeInitialShot());
  }

  private StateHandler makeInitialShot() {
    if (false) {
      ScoreSpeaker scoreSpeaker = new ScoreSpeaker(drive, shooter, () -> true);
      scoreSpeaker.schedule();
    }

    return this::nextSubState;
  }

  private StateHandler nextSubState() {
    if (currentSubState >= subStates.length) {
      return setDone();
    }

    return suspendForSubStateMachine(
        subStates[currentSubState++], subStateMachine -> AutoStateMachine.this::nextSubState);
  }
}
