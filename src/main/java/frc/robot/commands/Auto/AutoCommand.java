package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoCommand {

    public Command command;
    public String name;
    public Pose2d initPose;

    public AutoCommand(Command command, String name, Pose2d initPose){
        this.command = command;
        this.name = name;
        this.initPose = initPose;
    }
}