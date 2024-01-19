package frc.robot.autonomous1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoCommand {

  public Command command;
  public String name;
  public Pose2d initPose;

  public AutoCommand(String name, Command command, Pose2d initPose) {
    this.command = command;
    this.name = name;
    this.initPose = initPose;
  }
}
