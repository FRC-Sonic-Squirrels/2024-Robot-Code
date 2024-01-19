package frc.robot.autonomous;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoCommand {

  public Command command;
  public String name;
  public Pose2d initPose;
  public ChoreoTrajectory traj;

  public AutoCommand(String name, Command command, Pose2d initPose, ChoreoTrajectory traj) {
    this.command = command;
    this.name = name;
    this.initPose = initPose;
    this.traj = traj;
  }
}
