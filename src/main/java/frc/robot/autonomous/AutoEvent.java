package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoEvent {
  public Command command;
  public double timeSeconds;

  public AutoEvent(Command command, double timeSeconds) {
    this.command = command;
    this.timeSeconds = timeSeconds;
  }
}
