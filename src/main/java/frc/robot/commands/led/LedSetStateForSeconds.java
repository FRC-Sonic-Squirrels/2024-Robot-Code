// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.RobotState;

public class LedSetStateForSeconds extends Command {
  private final LED led;
  private final RobotState state;
  private final Timer timer = new Timer();
  private final double seconds;

  /** Creates a new LedSetStateForSeconds. */
  public LedSetStateForSeconds(LED led, RobotState state, double seconds) {
    this.led = led;
    this.state = state;
    this.seconds = seconds;

    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    led.setRobotState(state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.setRobotState(RobotState.BASE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > seconds;
  }
}
