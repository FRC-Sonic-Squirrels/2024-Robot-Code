// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomous.AutoCommand;
import java.util.function.Supplier;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private RobotContainer robotContainer;

  private LoggedDashboardChooser<Supplier<AutoCommand>> autonomousChooser = null;
  private AutoCommand lastAutoCommand = null;
  private String lastAutoName = null;
  private Alliance lastAlliance = null;
  private boolean hasEnteredTeleAtSomePoint = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.RobotMode.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick
        // Logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        // FIXME: add a git ignored folder where logs are saved
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> Logger.recordOutput("ActiveCommands/" + command.getName(), true));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> Logger.recordOutput("ActiveCommands/" + command.getName(), false));
    CommandScheduler.getInstance()
        .onCommandFinish(
            command -> Logger.recordOutput("ActiveCommands/" + command.getName(), false));

    // FIXME: remove this eventually
    Logger.recordOutput("TIME/CTRE TIME", Utils.getCurrentTimeSeconds());
    Logger.recordOutput("TIME/FPGA TIME", Timer.getFPGATimestamp());
    Logger.recordOutput("TIME/REAL FPGA", Logger.getRealTimestamp());

    Logger.recordOutput("RobotState/scoring mode", RobotState.getInstance().getScoringMode());

    robotContainer.updateVisualization();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // FIXME: need to remove max distance away from current estimate restriction for vision
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // handle choosing autonomous
    boolean shouldUpdateAutonomousCommand = false;
    if (autonomousChooser == null) {
      autonomousChooser = robotContainer.getAutonomousChooser();
    }

    var currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    var currentChooserSelectedName = autonomousChooser.getSendableChooser().getSelected();

    if (lastAlliance == null || lastAlliance != currentAlliance) {
      shouldUpdateAutonomousCommand = true;
    }

    if (lastAutoName == null
        || currentChooserSelectedName == null
        || !lastAutoName.equals(currentChooserSelectedName)) {
      shouldUpdateAutonomousCommand = true;
    }

    if (shouldUpdateAutonomousCommand) {
      // b/c chooser returns a supplier when we call get() on the supplier we get a update
      // trajectory & initial position for if our alliance has changed
      lastAutoCommand = autonomousChooser.get().get();
      lastAutoName = currentChooserSelectedName;
      lastAlliance = currentAlliance;

      // if FMS: only reset if we haven't entered tele. I.E only reset poses before match starts
      // if no FMS: always reset pose to auto pose.
      boolean shouldResetPose = false;
      if (DriverStation.isFMSAttached()) {
        if (!hasEnteredTeleAtSomePoint) {
          shouldResetPose = true;
        }
      } else {
        shouldResetPose = true;
      }

      if (shouldResetPose) {
        var pose =
            lastAlliance == Alliance.Blue
                ? lastAutoCommand.initPose
                : new Pose2d(
                    Constants.FieldConstants.FIELD_LENGTH - lastAutoCommand.initPose.getX(),
                    lastAutoCommand.initPose.getY(),
                    new Rotation2d(
                        -lastAutoCommand.initPose.getRotation().getCos(),
                        lastAutoCommand.initPose.getRotation().getSin()));

        robotContainer.setPose(pose);
      }

      Logger.recordOutput("Auto/SelectedAuto", lastAutoCommand.name);
      Logger.recordOutput("Auto/currentChooserValue", currentChooserSelectedName);
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    // schedule the autonomous command (example)
    if (lastAutoCommand != null) {
      lastAutoCommand.command.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (lastAutoCommand != null) {
      lastAutoCommand.command.cancel();
    }

    hasEnteredTeleAtSomePoint = true;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    robotContainer.updateRobotState();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(robotContainer.getCurrentDraws()));

    Logger.recordOutput("SimulatedRobot/currentDraws", robotContainer.getCurrentDraws());

    Logger.recordOutput("SimulatedRobot/batteryVoltage", RobotController.getBatteryVoltage());
  }
}
