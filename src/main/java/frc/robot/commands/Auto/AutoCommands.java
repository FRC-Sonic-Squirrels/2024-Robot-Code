package frc.robot.commands.Auto;

import java.util.Optional;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.Drivetrain;

public class AutoCommands {

    private Drivetrain drivetrain;
    public AutoCommands(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
    }

    private Command doNothing(){
        return new InstantCommand();
    }

    private Command testAuto(){
        ChoreoTrajectory traj = Choreo.getTrajectory("TestPath");
        return Choreo.choreoSwerveCommand(
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
    }

    public Command[] autoCommands(){
        return new Command[]{
            doNothing(),
            testAuto()
        };
    }
}
