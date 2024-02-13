// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import frc.lib.team2930.StateMachine;
import frc.robot.DrivetrainWrapper;
import frc.robot.commands.ScoreSpeaker;
import frc.robot.subsystems.shooter.Shooter;

public class AutoStateMachine extends StateMachine {
  public StateHandler[] handlers;
  private DrivetrainWrapper drive;
  private Shooter shooter;

  /** Creates a new AutoSubstateMachine. */
  public AutoStateMachine(
      AutoSubstateMachine[] subStates, DrivetrainWrapper drive, Shooter shooter) {
    this.drive = drive;
    this.shooter = shooter;
    handlers = new StateHandler[subStates.length];

    for (int i = subStates.length - 1; i >= 0; i--) {
      int j = i + 1;

      final int iFinal = i;

      if (i != subStates.length - 1) {
        handlers[i] =
            () -> {
              return suspendForSubStateMachine(
                  subStates[iFinal],
                  new ResumeStateHandler() {
                    @Override
                    public StateHandler advance(StateMachine subStateMachine) {
                      return handlers[j];
                    }
                  });
            };
      } else {
        handlers[i] =
            () -> {
              return suspendForSubStateMachine(
                  subStates[iFinal],
                  new ResumeStateHandler() {
                    @Override
                    public StateHandler advance(StateMachine subStateMachine) {
                      return setDone();
                    }
                  });
            };
      }
    }

    setInitialState(makeInitialShot());
  }

  private StateHandler makeInitialShot() {
    ScoreSpeaker scoreSpeaker = new ScoreSpeaker(drive, shooter, () -> true);
    // scoreSpeaker.schedule();
    return handlers[0];
  }
}
