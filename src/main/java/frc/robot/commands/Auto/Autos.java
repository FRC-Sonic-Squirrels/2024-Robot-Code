package frc.robot.commands.Auto;

import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;
import java.util.Optional;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.Drivetrain;

public class Autos {

    private Drivetrain drivetrain;
    public Autos(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
    }

    private AutoCommand doNothing(){
        return new AutoCommand(new InstantCommand(), "doNothing", new Pose2d(0,0,new Rotation2d(0)));
    }

    private AutoCommand testAuto(){
        return generateAutoCommand("testAuto");
    }

    public AutoCommand[] autoCommands(){
        return new AutoCommand[]{
            doNothing(),
            testAuto()
        };
    }

    private AutoCommand generateAutoCommand(String name, AutoEvent... events){
        ChoreoTrajectory traj = Choreo.getTrajectory(name);
        Command command = Choreo.choreoSwerveCommand(
            traj, 
            drivetrain::getPose, 
            AutoConstants.AUTO_X_PID, 
            AutoConstants.AUTO_Y_PID, 
            AutoConstants.AUTO_THETA_PID,
            (ChassisSpeeds speeds) ->
            drivetrain.runVelocity(speeds), 
            () -> {
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == Alliance.Red;
                }, 
            drivetrain);
        for (AutoEvent autoEvent : events) {
            command = command.alongWith(new SequentialCommandGroup(new WaitCommand(autoEvent.timeSeconds), autoEvent.command));
        }
        return new AutoCommand(command, name, traj.getInitialPose());
    }
}
