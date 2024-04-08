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

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.ArrayUtil;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team2930.commands.RunsWhenDisabledInstantCommand;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotMode.Mode;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.autonomous.AutosManager;
import frc.robot.autonomous.AutosManager.Auto;
import frc.robot.autonomous.AutosSubsystems;
import frc.robot.commands.AutoClimb;
import frc.robot.commands.LoadGamepieceToShooter;
import frc.robot.commands.ScoreSpeaker;
import frc.robot.commands.drive.DrivetrainDefaultTeleopDrive;
import frc.robot.commands.drive.RotateToAngle;
import frc.robot.commands.endEffector.EndEffectorCenterNoteBetweenToFs;
import frc.robot.commands.endEffector.EndEffectorPrepareNoteForTrap;
import frc.robot.commands.intake.IntakeEject;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.commands.led.LedSetStateForSeconds;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.commands.mechanism.arm.ArmSetAngle;
import frc.robot.commands.mechanism.elevator.ElevatorSetHeight;
import frc.robot.commands.shooter.ShooterScoreSpeakerStateMachine;
import frc.robot.configs.SimulatorRobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.BaseRobotState;
import frc.robot.subsystems.LED.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import frc.robot.subsystems.endEffector.EndEffectorIOReal;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionModuleConfiguration;
import frc.robot.subsystems.visionGamepiece.VisionGamepiece;
import frc.robot.subsystems.visionGamepiece.VisionGamepieceIO;
import frc.robot.subsystems.visionGamepiece.VisionGamepieceIOReal;
import frc.robot.subsystems.visionGamepiece.VisionGamepieceIOSim;
import frc.robot.visualization.ClimbVisualization;
import frc.robot.visualization.GamepieceVisualization;
import frc.robot.visualization.MechanismVisualization;
import frc.robot.visualization.SimpleMechanismVisualization;
import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final LoggerEntry.EnumValue<RobotType> logRobotType =
      LoggerGroup.root.buildEnum("RobotType");
  private static final LoggerEntry.EnumValue<Mode> logRobotMode =
      LoggerGroup.root.buildEnum("RobotMode");

  private final Drivetrain drivetrain;
  private final DrivetrainWrapper drivetrainWrapper;
  public final AprilTagFieldLayout aprilTagLayout;
  public final Vision vision;
  private final Arm arm;
  private final Elevator elevator;
  private final Intake intake;
  private final Shooter shooter;
  private final EndEffector endEffector;
  private final VisionGamepiece visionGamepiece;
  private final LED led;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private static final TunableNumberGroup groupTunable = new TunableNumberGroup("Speaker");
  private static final LoggedTunableNumber tunableX = groupTunable.build("dX", 50.0);
  private static final LoggedTunableNumber tunableY = groupTunable.build("dY", 0);
  private static final LoggedTunableNumber passThroughVel =
      groupTunable.build("PlopThroughVel", 5000);

  private static final LoggedTunableNumber passThroughPivotPitch =
      groupTunable.build("plopThroughPivotPitch", 12.0);

  private final ShuffleBoardLayouts shuffleBoardLayouts;

  private final LoggedDashboardChooser<String> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");
  private final HashMap<String, Supplier<Auto>> stringToAutoSupplierMap = new HashMap<>();
  private final AutosManager autoManager;

  private Trigger noteInRobot;
  private final Trigger twenty_Second_Warning;

  public DigitalInput breakModeButton = new DigitalInput(0);
  public DigitalInput homeSensorsButton = new DigitalInput(1);

  Trigger breakModeButtonTrigger =
      new Trigger(() -> !breakModeButton.get() && !DriverStation.isEnabled());

  Trigger homeSensorsButtonTrigger =
      new Trigger(() -> !homeSensorsButton.get() && !DriverStation.isEnabled());

  // private LoggedTunableNumber tunableX = new
  // LoggedTunableNumber("Localization/tunableXPose",
  // 0.0);
  // private LoggedTunableNumber tunableY = new
  // LoggedTunableNumber("Localization/tunableYPose",
  // 0.0);

  boolean brakeModeTriggered = true;

  ScoreSpeaker scoreSpeaker;
  AutoClimb autoClimb;

  private boolean is_teleop;
  private boolean is_autonomous;

  private boolean reactionArmsDown = false;

  private boolean brakeModeFailure = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    RobotType robotType = Constants.RobotMode.getRobot();
    Mode mode = Constants.RobotMode.getMode();

    logRobotType.info(robotType);
    logRobotMode.info(mode);

    var config = robotType.config.get();
    aprilTagLayout = config.getAprilTagFieldLayout();

    if (mode == Mode.REPLAY) {
      drivetrain =
          new Drivetrain(
              config,
              new GyroIO.Fake(),
              config.getReplaySwerveModuleObjects(),
              () -> is_autonomous);

      vision =
          new Vision(
              aprilTagLayout,
              drivetrain::getPoseEstimatorPose,
              drivetrain::getRotationGyroOnly,
              drivetrain::addVisionEstimate,
              config.getReplayVisionModules());

      arm = new Arm(new ArmIO() {});
      elevator = new Elevator(new ElevatorIO() {});
      intake = new Intake(new IntakeIO() {});
      shooter = new Shooter(new ShooterIO() {});
      endEffector = new EndEffector(new EndEffectorIO() {});
      visionGamepiece =
          new VisionGamepiece(
              new VisionGamepieceIO() {}, drivetrain::getPoseEstimatorPoseAtTimestamp);
      led =
          new LED(elevator::getHeightInches, () -> brakeModeTriggered, drivetrain::isGyroConnected);
    } else { // REAL and SIM robots HERE
      switch (robotType) {
        case ROBOT_SIMBOT_REAL_CAMERAS:
        case ROBOT_SIMBOT:
          com.ctre.phoenix6.unmanaged.Unmanaged.setPhoenixDiagnosticsStartTime(0.0);

          drivetrain =
              new Drivetrain(
                  config, new GyroIO.Fake(), config.getSwerveModuleObjects(), () -> is_autonomous);

          if (robotType == RobotType.ROBOT_SIMBOT_REAL_CAMERAS) {
            // Sim Robot, Real Cameras
            vision =
                new Vision(
                    aprilTagLayout,
                    drivetrain::getPoseEstimatorPose,
                    drivetrain::getRotationGyroOnly,
                    drivetrain::addVisionEstimate,
                    config.getVisionModuleObjects());

            visionGamepiece =
                new VisionGamepiece(
                    new VisionGamepieceIOReal(), drivetrain::getPoseEstimatorPoseAtTimestamp);

          } else {
            VisionModuleConfiguration[] visionModules = {
              VisionModuleConfiguration.buildSim(
                  SimulatorRobotConfig.SHOOTER_SIDE_LEFT_CAMERA_NAME,
                  SimulatorRobotConfig.SHOOTER_SIDE_LEFT,
                  config,
                  drivetrain::getPoseEstimatorPose),
              VisionModuleConfiguration.buildSim(
                  SimulatorRobotConfig.SHOOTER_SIDE_RIGHT_CAMERA_NAME,
                  SimulatorRobotConfig.SHOOTER_SIDE_RIGHT,
                  config,
                  drivetrain::getPoseEstimatorPose),
            };
            // Sim Cameras
            vision =
                new Vision(
                    aprilTagLayout,
                    drivetrain::getPoseEstimatorPose,
                    drivetrain::getRotationGyroOnly,
                    drivetrain::addVisionEstimate,
                    visionModules);

            visionGamepiece =
                new VisionGamepiece(
                    new VisionGamepieceIOSim(config, drivetrain::getPoseEstimatorPose),
                    drivetrain::getPoseEstimatorPoseAtTimestamp);
          }

          arm = new Arm(new ArmIOSim());
          elevator = new Elevator(new ElevatorIOSim());
          intake = new Intake(new IntakeIOSim());
          shooter = new Shooter(new ShooterIOSim());
          endEffector = new EndEffector(new EndEffectorIOSim());
          led =
              new LED(
                  elevator::getHeightInches, () -> brakeModeTriggered, drivetrain::isGyroConnected);
          break;

        case ROBOT_2023_RETIRED_ROBER:
          drivetrain =
              new Drivetrain(
                  config,
                  new GyroIOPigeon2(config),
                  config.getSwerveModuleObjects(),
                  () -> is_autonomous);

          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getPoseEstimatorPose,
                  drivetrain::getRotationGyroOnly,
                  drivetrain::addVisionEstimate,
                  config.getReplayVisionModules());
          arm = new Arm(new ArmIO() {});
          elevator = new Elevator(new ElevatorIO() {});
          intake = new Intake(new IntakeIO() {});
          shooter = new Shooter(new ShooterIO() {});
          endEffector = new EndEffector(new EndEffectorIO() {});
          visionGamepiece =
              new VisionGamepiece(
                  new VisionGamepieceIO() {}, drivetrain::getPoseEstimatorPoseAtTimestamp);

          led =
              new LED(
                  elevator::getHeightInches, () -> brakeModeTriggered, drivetrain::isGyroConnected);
          break;

        case ROBOT_2024_MAESTRO:
          // README: for development purposes, comment any of the real IO's you DON'T want
          // to use
          // and
          // uncomment the empty IO's as a replacement

          // -- All real IO's
          drivetrain =
              new Drivetrain(
                  config,
                  new GyroIOPigeon2(config),
                  config.getSwerveModuleObjects(),
                  () -> is_autonomous);

          // vision =
          // new Vision(
          // aprilTagLayout,
          // drivetrain::getPoseEstimatorPose,
          // drivetrain::addVisionEstimate,
          // config.getVisionModuleObjects());
          // visionGamepiece =
          // new VisionGamepiece(new VisionGamepieceIOReal(),
          // drivetrain::getPoseEstimatorPose);

          intake = new Intake(new IntakeIOReal());
          // elevator = new Elevator(new ElevatorIOReal());
          // arm = new Arm(new ArmIOReal());
          endEffector = new EndEffector(new EndEffectorIOReal());
          shooter = new Shooter(new ShooterIOReal());

          // -- All empty IO's
          // drivetrain =
          // new Drivetrain(config, new GyroIO() {},
          // config.getReplaySwerveModuleObjects());

          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getPoseEstimatorPose,
                  drivetrain::getRotationGyroOnly,
                  drivetrain::addVisionEstimate,
                  config.getVisionModuleObjects());

          visionGamepiece =
              new VisionGamepiece(
                  new VisionGamepieceIOReal(), drivetrain::getPoseEstimatorPoseAtTimestamp);

          // intake = new Intake(new IntakeIO() {});
          elevator = new Elevator(new ElevatorIOReal());
          arm = new Arm(new ArmIOReal());
          // endEffector = new EndEffector(new EndEffectorIO() {});
          // shooter = new Shooter(new ShooterIO() {});

          led =
              new LED(
                  elevator::getHeightInches, () -> brakeModeTriggered, drivetrain::isGyroConnected);
          break;

        default:
          drivetrain =
              new Drivetrain(
                  config,
                  new GyroIO.Fake(),
                  config.getReplaySwerveModuleObjects(),
                  () -> is_autonomous);
          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getPoseEstimatorPose,
                  drivetrain::getRotationGyroOnly,
                  drivetrain::addVisionEstimate,
                  config.getReplayVisionModules());
          arm = new Arm(new ArmIO() {});
          elevator = new Elevator(new ElevatorIO() {});
          intake = new Intake(new IntakeIO() {});
          shooter = new Shooter(new ShooterIO() {});
          endEffector = new EndEffector(new EndEffectorIO() {});
          visionGamepiece =
              new VisionGamepiece(
                  new VisionGamepieceIO() {}, drivetrain::getPoseEstimatorPoseAtTimestamp);

          led =
              new LED(
                  elevator::getHeightInches, () -> brakeModeTriggered, drivetrain::isGyroConnected);
          break;
      }
    }

    drivetrainWrapper = new DrivetrainWrapper(drivetrain);

    // FIXME: remove once we are happy with path planner based swerve
    // characterization
    AutoBuilder.configureHolonomic(
        drivetrain::getPoseEstimatorPose,
        drivetrain::setPose,
        drivetrain::getChassisSpeeds,
        drivetrainWrapper::setVelocityOverride,
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(0.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(0.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            config.getDriveBaseRadius(), // Drive base radius in meters.
            // Distance from robot center to
            // furthest module.
            new ReplanningConfig() // Default path replanning config. See the
            // API for the options
            // here
            ),
        () -> false,
        drivetrain);

    var subsystems =
        new AutosSubsystems(
            drivetrainWrapper, elevator, arm, intake, endEffector, shooter, visionGamepiece, led);

    autoManager = new AutosManager(subsystems, config, autoChooser, stringToAutoSupplierMap);

    drivetrain.setDefaultCommand(
        new DrivetrainDefaultTeleopDrive(
            drivetrainWrapper,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    shuffleBoardLayouts =
        new ShuffleBoardLayouts(arm, elevator, endEffector, intake, shooter, drivetrain);

    scoreSpeaker = new ScoreSpeaker(drivetrainWrapper, shooter, endEffector, () -> true);
    twenty_Second_Warning = new Trigger(() -> DriverStation.getMatchTime() > 115);

    noteInRobot =
        new Trigger(() -> endEffector.noteInEndEffector() || shooter.noteInShooter()).debounce(0.1);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ----------- DRIVER CONTROLS ------------

    // driverController
    //     .leftTrigger()
    //     .whileTrue(
    //         new DriveToGamepiece(
    //             led,
    //             visionGamepiece::getClosestGamepiece,
    //             drivetrainWrapper,
    //             endEffector::intakeSideTOFDetectGamepiece,
    //             () -> drivetrainWrapper.getPoseEstimatorPose(true)));

    driverController
        .back()
        .onTrue(
            Commands.runOnce(
                () -> {
                  Pose2d pose = drivetrain.getPoseEstimatorPose();
                  drivetrain.setPose(
                      new Pose2d(pose.getX(), pose.getY(), Constants.zeroRotation2d));
                },
                drivetrain));

    driverController
        .rightBumper()
        .whileTrue(
            new IntakeGamepiece(intake, endEffector, shooter, arm, elevator, (rumble) -> {})
                .finallyDo(
                    (interrupted) -> {
                      if (!interrupted)
                        CommandScheduler.getInstance()
                            .schedule(new LedSetStateForSeconds(led, RobotState.INTAKE_SUCCESS, 1));
                    }))
        .whileTrue(
            Commands.run(
                    () -> {
                      if (endEffector.intakeSideTOFDetectGamepiece()
                          || endEffector.shooterSideTOFDetectGamepiece()
                          || shooter.noteInShooter()) {
                        driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5);
                        led.setRobotState(RobotState.INTAKE_SUCCESS);
                      } else {
                        driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                        led.setRobotState(RobotState.BASE);
                      }
                    })
                .finallyDo(
                    () -> {
                      driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                      led.setRobotState(RobotState.BASE);
                    }))
        // .onFalse(
        //     new EndEffectorCenterNoteBetweenToFs(endEffector, intake, shooter)
        //         .withTimeout(2.0));

        .onFalse(
            Commands.run(
                    () -> {
                      endEffector.setVelocity(500);
                      intake.setVelocity(500);
                      shooter.setKickerVelocity(500);
                    },
                    endEffector,
                    intake,
                    shooter)
                .until(
                    () ->
                        !endEffector.intakeSideTOFDetectGamepiece()
                            && endEffector.shooterSideTOFDetectGamepiece())
                .withTimeout(4.0)
                .andThen(
                    new IntakeEject(shooter, endEffector, intake)
                        .until(() -> !endEffector.shooterSideTOFDetectGamepiece()))
                .withTimeout(1.5)
                .andThen(new EndEffectorCenterNoteBetweenToFs(endEffector, intake, shooter))
                .withTimeout(2.0));

    Supplier<Command> toggleReactionArms =
        () ->
            new ConditionalCommand(
                Commands.runOnce(
                    () -> {
                      elevator.retractReactionArms();
                      reactionArmsDown = false;
                    },
                    elevator),
                Commands.runOnce(
                    () -> {
                      elevator.deployReactionArms();
                      reactionArmsDown = true;
                    },
                    elevator),
                () -> reactionArmsDown);

    driverController
        .povUp()
        .whileTrue(
            CommandComposer.stageAlignFast(
                aprilTagLayout,
                drivetrainWrapper,
                led,
                (r) -> driverController.getHID().setRumble(RumbleType.kBothRumble, r),
                visionGamepiece,
                elevator,
                arm,
                endEffector,
                shooter,
                intake,
                () -> reactionArmsDown,
                toggleReactionArms));

    driverController
        .povLeft()
        .whileTrue(CommandComposer.driveToChain(aprilTagLayout, drivetrainWrapper, led));

    driverController
        .rightTrigger()
        .whileTrue(
            ShooterScoreSpeakerStateMachine.getAsCommand(
                    drivetrainWrapper,
                    shooter,
                    endEffector,
                    intake,
                    led,
                    1000,
                    () -> true,
                    (rumble) -> driverController.getHID().setRumble(RumbleType.kBothRumble, rumble))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    driverController
        .leftBumper()
        .onTrue(
            CommandComposer.scoreAmp(
                endEffector,
                drivetrainWrapper,
                elevator,
                arm,
                intake,
                shooter,
                led,
                true,
                driverController.a(),
                (r) -> driverController.getHID().setRumble(RumbleType.kBothRumble, r)))
        // .whileTrue(
        //     new RotateToAngle(
        //         drivetrainWrapper,
        //         () -> Rotation2d.fromDegrees(90),
        //         () -> drivetrainWrapper.getPoseEstimatorPose(true)))
        .onFalse(
            CommandComposer.cancelScoreAmp(drivetrainWrapper, endEffector, elevator, arm, led));

    driverController
        .leftTrigger()
        .whileTrue(
            new RotateToAngle(
                drivetrainWrapper,
                () ->
                    drivetrainWrapper
                        .getGlobalPoseEstimatorPose(true)
                        .getTranslation()
                        .minus(Constants.FieldConstants.getSweepTargetTranslation())
                        .getAngle(),
                () -> drivetrainWrapper.getGlobalPoseEstimatorPose(true)));

    driverController
        .x()
        .onTrue(
            Commands.run(
                () -> {
                  shooter.setLauncherRPM(8700, 8000);
                  if (shooter.getRPM() > 8500) {
                    shooter.setKickerVelocity(passThroughVel.get());
                    endEffector.setVelocity(passThroughVel.get());
                    intake.setVelocity(passThroughVel.get());
                  }
                  shooter.setPivotPosition(Rotation2d.fromDegrees(passThroughPivotPitch.get()));
                },
                shooter,
                endEffector,
                intake))
        .onFalse(
            Commands.runOnce(
                () -> {
                  shooter.setPercentOut(0);
                  shooter.setKickerPercentOut(0.0);
                  endEffector.setPercentOut(0.0);
                  intake.setPercentOut(0.0);
                  shooter.setPivotPosition(ShooterConstants.Pivot.SHOOTER_STOW_PITCH);
                },
                shooter,
                endEffector,
                intake));

    driverController.povDown().onTrue(MechanismActions.loadingPosition(elevator, arm));

    driverController
        .y()
        .whileTrue(
            ShooterScoreSpeakerStateMachine.getAsCommand(
                drivetrainWrapper,
                shooter,
                endEffector,
                intake,
                led,
                5.0,
                () -> true,
                (r) -> {},
                true,
                Constants.ShooterConstants.Pivot.MAX_ANGLE_RAD.minus(Rotation2d.fromDegrees(3.0))));

    // driverController
    //     .povRight()
    //     .whileTrue(
    //         new DrivetrainDefaultTeleopDrive(drivetrainWrapper, () -> 1.0, () -> 0.0, () ->
    // 0.0));

    // driverController
    //     .povDown()
    //     .whileTrue(
    //         new DrivetrainDefaultTeleopDrive(drivetrainWrapper, () -> -1.0, () -> 0.0, () ->
    // 0.0));

    // ---------- OPERATOR CONTROLS -----------
    DoubleSupplier elevatorDelta =
        () -> MathUtil.applyDeadband(-operatorController.getLeftY() * 1, 0.3) / 5.0;
    DoubleSupplier armDelta =
        () -> MathUtil.applyDeadband(operatorController.getRightY() * -1, 0.3) * 10.0;

    operatorController
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                new ElevatorSetHeight(
                    elevator,
                    () ->
                        Units.Inches.of(elevator.getHeightInches() + elevatorDelta.getAsDouble())),
                new ArmSetAngle(
                    arm,
                    () -> arm.getAngle().plus(Rotation2d.fromDegrees(armDelta.getAsDouble())))));

    operatorController.leftBumper().onTrue(toggleReactionArms.get());

    operatorController
        .povUp()
        .onTrue(
            CommandComposer.prepMechForClimb(
                elevator,
                arm,
                endEffector,
                shooter,
                intake,
                () -> reactionArmsDown,
                toggleReactionArms));
    operatorController
        .povLeft()
        .onTrue(CommandComposer.autoClimb(elevator, arm, endEffector, shooter, intake));
    operatorController.povDown().onTrue(MechanismActions.climbDownPosition(elevator, arm));
    operatorController.povRight().onTrue(MechanismActions.climbTrapPosition(elevator, arm));

    operatorController.x().onTrue(MechanismActions.climbFinalRestPosition(elevator, arm));

    operatorController
        .y()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      shooter.setLauncherRPM(1000);
                      shooter.setKickerVelocity(passThroughVel.get());
                      intake.setVelocity(passThroughVel.get());
                      shooter.setPivotPosition(Rotation2d.fromDegrees(passThroughPivotPitch.get()));
                    },
                    shooter,
                    endEffector,
                    intake)
                .andThen(
                    elevator
                        .setReactionArmsRotationsCMD(
                            ElevatorConstants.ReactionArmConstants.REACTION_ARM_AMP_ROTATIONS)
                        .andThen(MechanismActions.noteGetOut(elevator, arm))))
        .onFalse(
            Commands.runOnce(
                    () -> {
                      shooter.setPercentOut(0);
                      shooter.setKickerPercentOut(0.0);
                      intake.setPercentOut(0.0);
                      shooter.setPivotPosition(ShooterConstants.Pivot.SHOOTER_STOW_PITCH);
                    },
                    shooter,
                    endEffector,
                    intake)
                .andThen(MechanismActions.loadingPosition(elevator, arm))
                .andThen(
                    elevator.setReactionArmsRotationsCMD(
                        ElevatorConstants.ReactionArmConstants.REACTION_ARM_HOME_ROTATIONS)));

    operatorController
        .b()
        .onTrue(Commands.runOnce(() -> endEffector.setVelocity(2500), endEffector))
        .onFalse(Commands.runOnce(() -> endEffector.setPercentOut(0.0), endEffector));

    operatorController
        .a()
        .whileTrue(new EndEffectorCenterNoteBetweenToFs(endEffector, intake, shooter));

    operatorController
        .rightBumper()
        .onTrue(new LoadGamepieceToShooter(shooter, endEffector, intake));

    operatorController.rightTrigger().whileTrue(new IntakeEject(shooter, endEffector, intake));

    operatorController.start().onTrue(new EndEffectorPrepareNoteForTrap(endEffector));

    // --- SIM ONLY --

    if (!Robot.isReal()) {
      operatorController
          .back()
          .onTrue(Commands.runOnce(() -> endEffector.markStartOfNoteIntaking()));

      operatorController
          .start()
          .whileTrue(
              Commands.run(
                  () ->
                      drivetrain.setPose(
                          new Pose2d(
                              drivetrain.getPoseEstimatorPose().getX() + 0.25,
                              drivetrain.getPoseEstimatorPose().getY(),
                              drivetrain.getPoseEstimatorPose().getRotation()))));
    }

    // -- buttons on robot
    homeSensorsButtonTrigger.onTrue(
        Commands.runOnce(elevator::resetSensorToHomePosition, elevator)
            .andThen(arm::resetSensorToHomePosition, arm)
            .andThen(shooter::pivotResetHomePosition, shooter)
            .andThen(elevator::resetReactionArmPositions)
            .andThen(new LedSetStateForSeconds(led, RobotState.HOME_SUBSYSTEMS, 1.5))
            .ignoringDisable(true));

    breakModeButtonTrigger.onTrue(
        new ConditionalCommand(
            Commands.runOnce(
                    () -> {
                      boolean armSuccess = arm.setNeutralMode(NeutralModeValue.Coast);
                      boolean elevatorSuccess = elevator.setNeutralMode(NeutralModeValue.Coast);
                      boolean shooterSuccess = shooter.setNeutralMode(NeutralModeValue.Coast);
                      elevator.setReactionArmIdleMode(IdleMode.kCoast);

                      brakeModeFailure = !armSuccess || !elevatorSuccess || !shooterSuccess;

                      //   drivetrain.setNeturalMode(NeutralModeValue.Coast);
                      brakeModeTriggered = false;
                    },
                    elevator,
                    arm)
                .andThen(
                    new LedSetStateForSeconds(
                        led,
                        brakeModeFailure ? RobotState.BRAKE_MODE_FAILED : RobotState.BREAK_MODE_OFF,
                        1.5))
                .ignoringDisable(true),
            Commands.runOnce(
                    () -> {
                      boolean armSuccess = arm.setNeutralMode(NeutralModeValue.Brake);
                      boolean elevatorSuccess = elevator.setNeutralMode(NeutralModeValue.Brake);
                      boolean shooterSuccess = shooter.setNeutralMode(NeutralModeValue.Brake);
                      elevator.setReactionArmIdleMode(IdleMode.kBrake);

                      brakeModeFailure = !armSuccess || !elevatorSuccess || !shooterSuccess;
                      //   drivetrain.setNeturalMode(NeutralModeValue.Brake);
                      brakeModeTriggered = true;
                    },
                    elevator,
                    arm)
                .andThen(
                    new LedSetStateForSeconds(
                        led,
                        brakeModeFailure ? RobotState.BRAKE_MODE_FAILED : RobotState.BREAK_MODE_ON,
                        1.5))
                .ignoringDisable(true),
            () -> brakeModeTriggered));

    twenty_Second_Warning.onTrue(
        new LedSetStateForSeconds(led, RobotState.TWENTY_SECOND_WARNING, 0.5));

    // Add Reset and Reboot buttons to SmartDashboard
    SmartDashboard.putData(
        "PV Restart SW 1_Shooter_Left",
        new RunsWhenDisabledInstantCommand(() -> Vision.restartPhotonVision("10.29.30.13")));

    SmartDashboard.putData(
        "PV REBOOT 1_Shooter_Left",
        new RunsWhenDisabledInstantCommand(() -> Vision.rebootPhotonVision("10.29.30.13")));

    SmartDashboard.putData(
        "PV Restart SW 2_Shooter_Right",
        new RunsWhenDisabledInstantCommand(() -> Vision.restartPhotonVision("10.29.30.14")));

    SmartDashboard.putData(
        "PV REBOOT 2_Shooter_Right",
        new RunsWhenDisabledInstantCommand(() -> Vision.rebootPhotonVision("10.29.30.14")));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public LoggedDashboardChooser<String> getAutonomousChooser() {
    return autoChooser;
  }

  public Supplier<Auto> getAutoSupplierForString(String string) {
    return stringToAutoSupplierMap.getOrDefault(string, autoManager::doNothing);
  }

  public void setPose(Pose2d pose) {
    drivetrain.setPose(pose);
  }

  public void matchRawOdometryToPoseEstimatorValue() {
    drivetrain.setRawOdometryPose(drivetrain.getPoseEstimatorPose());
  }

  public double[] getCurrentDraws() {
    double[] current =
        new double[] {
          intake.getCurrentDraw(),
        };

    current = ArrayUtil.concatWithArrayCopy(current, drivetrain.getCurrentDrawAmps());
    return current;
  }

  public void applyToDrivetrain() {
    drivetrainWrapper.apply();
  }

  public void enterDisabled() {
    // FIXME: need to remove max distance away from current estimate restriction for
    // vision
    resetSubsystems();
    vision.useMaxDistanceAwayFromExistingEstimate(false);
    vision.useGyroBasedFilteringForVision(false);

    is_teleop = false;
    is_autonomous = false;
  }

  public void enterAutonomous() {
    // setBrakeMode();
    vision.useMaxDistanceAwayFromExistingEstimate(true);
    vision.useGyroBasedFilteringForVision(true);

    visionGamepiece.setPipelineIndex(0);

    is_teleop = false;
    is_autonomous = true;
  }

  public void enterTeleop() {
    // setBrakeMode();
    resetDrivetrainResetOverrides();
    vision.useMaxDistanceAwayFromExistingEstimate(true);
    vision.useGyroBasedFilteringForVision(true);

    led.setBaseRobotState(BaseRobotState.NOTE_STATUS);

    visionGamepiece.setPipelineIndex(1);

    is_teleop = true;
    is_autonomous = false;
  }

  public void resetDrivetrainResetOverrides() {
    drivetrainWrapper.resetVelocityOverride();
    drivetrainWrapper.resetRotationOverride();
  }

  public void updateVisualization() {
    // Disable visualization for real robot
    if (Robot.isReal()) return;

    SimpleMechanismVisualization.updateVisualization(
        Constants.zeroRotation2d,
        shooter.getPitch(),
        Math.hypot(
            GeometryUtil.getDist(
                drivetrain.getPoseEstimatorPose().getTranslation(),
                Constants.FieldConstants.getSpeakerTranslation()),
            Constants.FieldConstants.SPEAKER_HEIGHT.in(Units.Meters)),
        shooter.getRPM(),
        elevator.getHeightInches());

    SimpleMechanismVisualization.logMechanism();

    MechanismVisualization.updateVisualization(
        arm.getAngle(),
        shooter.getPitch(),
        elevator.getHeightInches(),
        endEffector.intakeSideTOFDetectGamepiece() || endEffector.shooterSideTOFDetectGamepiece());

    MechanismVisualization.logMechanism();

    GamepieceVisualization.getInstance().logTraj();

    ClimbVisualization climbVisualization = ClimbVisualization.getInstance();
    climbVisualization.updateVisualization(drivetrainWrapper.getPoseEstimatorPose(true));
    climbVisualization.logPose();
  }

  public void resetSubsystems() {
    elevator.resetSubsystem();
    arm.resetSubsystem();
  }

  public void setBrakeMode() {
    brakeModeTriggered = true;
    elevator.setNeutralMode(NeutralModeValue.Brake);
    elevator.setReactionArmIdleMode(IdleMode.kBrake);
    arm.setNeutralMode(NeutralModeValue.Brake);
    shooter.setNeutralMode(NeutralModeValue.Brake);
  }

  public void updateLedGamepieceState() {
    if (noteInRobot.getAsBoolean() && !led.getNoteStatus()) {
      led.setNoteStatus(true);
    }
    if (!noteInRobot.getAsBoolean() && led.getNoteStatus()) {
      led.setNoteStatus(false);
    }
  }
}
