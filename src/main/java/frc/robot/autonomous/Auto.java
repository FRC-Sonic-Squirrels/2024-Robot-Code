package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class Auto {

  public Command command;
  public String name;
  public Pose2d initPose;

  public Auto(String name, Command command, Pose2d initPose) {
    this.command = command;
    this.name = name;
    this.initPose = initPose;
  }
}
