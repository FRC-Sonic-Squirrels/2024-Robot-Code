// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import frc.lib.team2930.StateMachine;

public class AutoStateMachine extends StateMachine {
  public StateHandler[] handlers;

  /** Creates a new AutoSubstateMachine. */
  public AutoStateMachine(AutoSubstateMachine[] subStates) {
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

    setInitialState(handlers[0]);
  }
}
