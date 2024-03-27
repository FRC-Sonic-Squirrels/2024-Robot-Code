// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.robotStates;

public class LedSetStateForSeconds extends Command {
  private final LED led;
  private final robotStates state;
  private robotStates previousState;
  private final boolean returnToPrevLedState;
  private final Timer timer = new Timer();
  private final double seconds;

  /** Creates a new LedSetStateForSeconds. */
  public LedSetStateForSeconds(LED led, robotStates state, double seconds) {
    this(led, state, seconds, false);
  }

  public LedSetStateForSeconds(
      LED led, robotStates state, double seconds, boolean returnToPrevLedState) {
    this.led = led;
    this.state = state;
    this.seconds = seconds;
    this.returnToPrevLedState = returnToPrevLedState;

    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    previousState = led.getCurrentState();
    led.setRobotState(state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.setRobotState(returnToPrevLedState ? previousState : robotStates.NOTHING);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > seconds;
  }
}
