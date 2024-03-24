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
  private final DrivetrainWrapper drive;
  private final Shooter shooter;
  private final EndEffector endEffector;
  private final Elevator elevator;
  private final Arm arm;
  private final Intake intake;
  private final VisionGamepiece visionGamepiece;
  private final RobotConfig config;
  private final ChoreoTrajectoryWithName[] intakingTrajs;
  private final ChoreoTrajectoryWithName[] shootingTrajs;
  private final Boolean[] useVision;
  private final StateMachine[] overrideStateMachines;
  private int currentSubState;
  private double initShootDeadline;
  private boolean shootingDone;

  public AutoStateMachine(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      Elevator elevator,
      Arm arm,
      Intake intake,
      VisionGamepiece visionGamepiece,
      StateMachine[] overrideStateMachines,
      double initShootDeadline,
      RobotConfig config) {
    this(
        drive,
        shooter,
        endEffector,
        elevator,
        arm,
        intake,
        visionGamepiece,
        null,
        initShootDeadline,
        config,
        overrideStateMachines);
  }

  public AutoStateMachine(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      Elevator elevator,
      Arm arm,
      Intake intake,
      VisionGamepiece visionGamepiece,
      List<PathDescriptor> subStateTrajNames,
      double initShootDeadline,
      RobotConfig config) {
    this(
        drive,
        shooter,
        endEffector,
        elevator,
        arm,
        intake,
        visionGamepiece,
        subStateTrajNames,
        initShootDeadline,
        config,
        null);
  }

  /** Creates a new AutoSubstateMachine. */
  private AutoStateMachine(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      Elevator elevator,
      Arm arm,
      Intake intake,
      VisionGamepiece visionGamepiece,
      List<PathDescriptor> subStateTrajNames,
      double initShootDeadline,
      RobotConfig config,
      StateMachine[] overrideStateMachines) {
    super("Auto");

    this.drive = drive;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;
    this.visionGamepiece = visionGamepiece;
    this.config = config;
    this.initShootDeadline = initShootDeadline;
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
        ShooterScoreSpeakerStateMachine.getAsCommand(drive, shooter, endEffector, intake, 5);

    return suspendForCommand(scoreSpeaker, this::shotDone);
  }

  private StateHandler shotDone(Command command) {
    shootingDone = true;
    return nextSubState(true);
  }

  private StateHandler nextSubState(boolean followPath) {
    StateHandler nextState;
    if (overrideStateMachines == null) {
      if (++currentSubState >= intakingTrajs.length) {
        return setDone();
      }

      if (followPath) {
        nextState =
            suspendForSubStateMachine(
                new AutoSubstateMachineChoreo(
                    drive,
                    shooter,
                    endEffector,
                    intake,
                    config,
                    elevator,
                    arm,
                    useVision[currentSubState],
                    intakingTrajs[currentSubState],
                    shootingTrajs[currentSubState],
                    visionGamepiece::getClosestGamepiece,
                    intakingTrajs[currentSubState].getFinalPose(true).getTranslation()),
                subStateMachine -> () -> this.nextSubState(!subStateMachine.wasStopped()));
      } else {
        nextState =
            suspendForSubStateMachine(
                new AutoSubstateMachineDriveTranslation(
                    drive,
                    shooter,
                    endEffector,
                    intake,
                    config,
                    elevator,
                    arm,
                    useVision[currentSubState],
                    intakingTrajs[currentSubState].getFinalPose(true).getTranslation(),
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

    return stateWithName(String.format("State %d", currentSubState), nextState);
  }
}
