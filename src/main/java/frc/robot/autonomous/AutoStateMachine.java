// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.StateMachine;
import frc.robot.autonomous.substates.AutoSubstateMachineChoreo;
import frc.robot.autonomous.substates.AutoSubstateMachineDriveTranslation;
import frc.robot.commands.shooter.ShooterScoreSpeakerStateMachine;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.visionGamepiece.VisionGamepiece;
import java.util.ArrayList;
import java.util.List;

public class AutoStateMachine extends StateMachine {
  private Command scoreSpeaker;
  private final AutosSubsystems subsystems;
  private final DrivetrainWrapper drive;
  private final Shooter shooter;
  private final EndEffector endEffector;
  private final Elevator elevator;
  private final Arm arm;
  private final Intake intake;
  private final VisionGamepiece visionGamepiece;
  private final LED led;
  private final RobotConfig config;
  private final ChoreoTrajectoryWithName[] intakingTrajs;
  private final ChoreoTrajectoryWithName[] shootingTrajs;
  private final Boolean[] useVision;
  private final StateMachine[] overrideStateMachines;
  private int currentSubState;

  public AutoStateMachine(
      AutosSubsystems subsystems, RobotConfig config, StateMachine[] overrideStateMachines) {
    this(subsystems, config, null, overrideStateMachines);
  }

  public AutoStateMachine(
      AutosSubsystems subsystems, RobotConfig config, List<PathDescriptor> subStateTrajNames) {
    this(subsystems, config, subStateTrajNames, null);
  }

  /** Creates a new AutoSubstateMachine. */
  private AutoStateMachine(
      AutosSubsystems subsystems,
      RobotConfig config,
      List<PathDescriptor> subStateTrajNames,
      StateMachine[] overrideStateMachines) {
    super("Auto");

    this.subsystems = subsystems;
    this.drive = subsystems.drivetrain();
    this.shooter = subsystems.shooter();
    this.endEffector = subsystems.endEffector();
    this.elevator = subsystems.elevator();
    this.arm = subsystems.arm();
    this.intake = subsystems.intake();
    this.visionGamepiece = subsystems.visionGamepiece();
    this.led = subsystems.led();
    this.config = config;
    this.currentSubState = -1;
    this.overrideStateMachines = overrideStateMachines;

    if (subStateTrajNames == null) {
      subStateTrajNames = new ArrayList<>();
    }

    intakingTrajs = new ChoreoTrajectoryWithName[subStateTrajNames.size()];
    shootingTrajs = new ChoreoTrajectoryWithName[subStateTrajNames.size()];
    useVision = new Boolean[subStateTrajNames.size()];

    for (int i = 0; i < subStateTrajNames.size(); i++) {
      var path = subStateTrajNames.get(i);
      useVision[i] = path.useVision();
      intakingTrajs[i] = ChoreoTrajectoryWithName.getTrajectory(path.intakingTraj());
      shootingTrajs[i] = ChoreoTrajectoryWithName.getTrajectory(path.shootingTraj());
    }

    setInitialState(stateWithName("autoInitialState", this::autoInitialState));
  }

  private StateHandler autoInitialState() {
    scoreSpeaker =
        ShooterScoreSpeakerStateMachine.getAsCommand(drive, shooter, endEffector, intake, led, 5);

    return suspendForCommand(scoreSpeaker, this::shotDone);
  }

  private StateHandler shotDone(Command command) {
    return nextSubState(true);
  }

  private StateHandler nextSubState(boolean followPath) {
    StateHandler nextState;
    if (overrideStateMachines == null) {
      if (++currentSubState >= intakingTrajs.length) {
        return setDone();
      }

      var targetGPPose = intakingTrajs[currentSubState].getFinalPose(true).getTranslation();

      if (followPath) {
        nextState =
            suspendForSubStateMachine(
                new AutoSubstateMachineChoreo(
                    subsystems,
                    config,
                    useVision[currentSubState],
                    intakingTrajs[currentSubState],
                    shootingTrajs[currentSubState],
                    visionGamepiece::getClosestGamepiece,
                    targetGPPose),
                subStateMachine -> () -> this.nextSubState(!subStateMachine.wasStopped()));
      } else {
        nextState =
            suspendForSubStateMachine(
                new AutoSubstateMachineDriveTranslation(
                    subsystems,
                    config,
                    useVision[currentSubState],
                    targetGPPose,
                    shootingTrajs[currentSubState],
                    visionGamepiece::getClosestGamepiece),
                subStateMachine -> () -> this.nextSubState(!subStateMachine.wasStopped()));
      }

    } else {
      if (++currentSubState >= overrideStateMachines.length) {
        return setDone();
      }

      nextState =
          suspendForSubStateMachine(
              overrideStateMachines[currentSubState],
              subStateMachine -> () -> this.nextSubState(false));
    }

    return stateWithName("State " + currentSubState, nextState);
  }
}
