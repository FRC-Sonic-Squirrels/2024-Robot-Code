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
  private ChoreoHelper initialPathChoreoHelper;
  private final ChoreoTrajectoryWithName initPath;
  private int currentSubState;
  private final boolean plopFirstGP;

  public AutoStateMachine(
      AutosSubsystems subsystems, RobotConfig config, StateMachine[] overrideStateMachines) {
    this(subsystems, config, false, null, false, null, overrideStateMachines);
  }

  public AutoStateMachine(
      AutosSubsystems subsystems,
      RobotConfig config,
      boolean doInitialPath,
      String initPath,
      StateMachine[] overrideStateMachines) {
    this(subsystems, config, doInitialPath, initPath, false, null, overrideStateMachines);
  }

  public AutoStateMachine(
      AutosSubsystems subsystems, RobotConfig config, List<PathDescriptor> subStateTrajNames) {
    this(subsystems, config, false, null, false, subStateTrajNames, null);
  }

  public AutoStateMachine(
      AutosSubsystems subsystems,
      RobotConfig config,
      boolean plopFirstGamepiece,
      List<PathDescriptor> subStateTrajNames) {
    this(subsystems, config, false, null, plopFirstGamepiece, subStateTrajNames, null);
  }

  public AutoStateMachine(
      AutosSubsystems subsystems,
      RobotConfig config,
      boolean doInitialPath,
      String initPath,
      List<PathDescriptor> subStateTrajNames) {
    this(subsystems, config, doInitialPath, initPath, false, subStateTrajNames, null);
  }

  /** Creates a new AutoSubstateMachine. */
  private AutoStateMachine(
      AutosSubsystems subsystems,
      RobotConfig config,
      boolean doInitDrive,
      String initPath,
      boolean plopFirstGP,
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
    this.plopFirstGP = plopFirstGP;

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

    if (doInitDrive) {
      this.initPath = ChoreoTrajectoryWithName.getTrajectory(initPath);
      setInitialState(stateWithName("driveOutState", this::driveOutStateInit));
    } else if (plopFirstGP) {
      this.initPath = null;
      setInitialState(() -> nextSubState(true));
    } else {
      this.initPath = null;
      setInitialState(stateWithName("autoInitialState", this::autoInitialState));
    }
  }

  private StateHandler driveOutStateInit() {
    initialPathChoreoHelper =
        new ChoreoHelper(
            timeFromStart(),
            drive.getPoseEstimatorPose(true),
            initPath,
            config.getDriveBaseRadius() / 2,
            2.0,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController());
    return stateWithName("driveOutState", this::driveOutState);
  }

  private StateHandler driveOutState() {
    var result =
        initialPathChoreoHelper.calculateChassisSpeeds(
            drive.getPoseEstimatorPose(true), timeFromStart());
    if (result.atEndOfPath()) {
      drive.resetVelocityOverride();
      return stateWithName("autoInitialState", this::autoInitialState);
    }
    drive.setVelocityOverride(result.chassisSpeeds());
    return null;
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
                    targetGPPose,
                    currentSubState == 1 && plopFirstGP),
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
