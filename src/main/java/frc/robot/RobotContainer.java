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

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.team2930.ArrayUtil;
import frc.robot.Constants.RobotMode.Mode;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.commands.arm.ArmSetAngle;
import frc.robot.commands.drive.DrivetrainDefaultTeleopDrive;
import frc.robot.commands.intake.EjectGamepiece;
import frc.robot.commands.intake.IntakeDefaultIdleRPM;
import frc.robot.configs.SimulatorRobotConfig;
import frc.robot.mechanismVisualization.SimpleMechanismVisualization;
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
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.limelight.LimelightIO;
import frc.robot.subsystems.limelight.LimelightIOReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionModule;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOReal;
import frc.robot.subsystems.wrist.WristIOSim;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain drivetrain;
  private final Vision vision;
  private final Arm arm;
  private final Elevator elevator;
  private final Intake intake;
  private final Shooter shooter;
  private final Wrist wrist;
  private final EndEffector endEffector;
  private final Limelight limelight;

  private final CommandXboxController controller = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    RobotType robotType = Constants.RobotMode.getRobot();
    Mode mode = Constants.RobotMode.getMode();

    Logger.recordOutput("RobotType", robotType);
    Logger.recordOutput("RobotMode", mode);

    var config = robotType.config.get();
    AprilTagFieldLayout aprilTagLayout;

    try {
      aprilTagLayout = config.getAprilTagFieldLayout();
    } catch (Exception e) {
      // FIXME:
      aprilTagLayout = new AprilTagFieldLayout(new ArrayList<AprilTag>(), 0.0, 0.0);
    }

    if (mode == Mode.REPLAY) {
      drivetrain = new Drivetrain(config, new GyroIO() {}, config.getReplaySwerveModuleObjects());
      vision = new Vision(aprilTagLayout, drivetrain, config.getReplayVisionModules());
      arm = new Arm(new ArmIO() {});
      elevator = new Elevator(new ElevatorIO() {});
      intake = new Intake(new IntakeIO() {});
      shooter = new Shooter(new ShooterIO() {});
      wrist = new Wrist(new WristIO() {});
      endEffector = new EndEffector(new EndEffectorIO() {});
      limelight = new Limelight(new LimelightIO() {}, drivetrain::getPose);

    } else { // REAL and SIM robots HERE
      switch (robotType) {
        case ROBOT_DEFAULT:
          drivetrain = null;
          vision = null;
          arm = null;
          elevator = null;
          intake = null;
          shooter = null;
          wrist = null;
          endEffector = null;
          limelight = null;
          break;

        case ROBOT_SIMBOT:
          com.ctre.phoenix6.unmanaged.Unmanaged.setPhoenixDiagnosticsStartTime(0.0);

          drivetrain = new Drivetrain(config, new GyroIO() {}, config.getSwerveModuleObjects());
          VisionModule[] visionModules = {
            new VisionModule(
                new VisionIOSim(
                    config,
                    drivetrain::getPose,
                    SimulatorRobotConfig.FRONT_LEFT_ROBOT_TO_CAMERA,
                    SimulatorRobotConfig.FRONT_LEFT_CAMERA_NAME),
                SimulatorRobotConfig.FRONT_LEFT_CAMERA_NAME,
                SimulatorRobotConfig.FRONT_LEFT_ROBOT_TO_CAMERA),
            new VisionModule(
                new VisionIOSim(
                    config,
                    drivetrain::getPose,
                    SimulatorRobotConfig.FRONT_RIGHT_ROBOT_TO_CAMERA,
                    SimulatorRobotConfig.FRONT_RIGHT_CAMERA_NAME),
                SimulatorRobotConfig.FRONT_RIGHT_CAMERA_NAME,
                SimulatorRobotConfig.FRONT_RIGHT_ROBOT_TO_CAMERA),
            // new VisionModule(
            //     new VisionIOSim(
            //         config,
            //         drivetrain::getPose,
            //         SimulatorRobotConfig.BACK_ROBOT_TO_CAMERA,
            //         SimulatorRobotConfig.BACK_CAMERA_NAME),
            //     SimulatorRobotConfig.BACK_CAMERA_NAME,
            //     SimulatorRobotConfig.BACK_ROBOT_TO_CAMERA),
          };

          // vision = new Vision(aprilTagLayout, drivetrain, visionModules);

          // TO remove sim vision uncomment this
          VisionModule visionModule =
              new VisionModule(new VisionIO() {}, "empty", new Transform3d());
          vision = new Vision(aprilTagLayout, drivetrain, visionModule);
          // TO remove sim vision uncomment this

          arm = new Arm(new ArmIOSim());
          elevator = new Elevator(new ElevatorIOSim());
          intake = new Intake(new IntakeIOSim());
          shooter = new Shooter(new ShooterIOSim());
          wrist = new Wrist(new WristIOSim());
          endEffector = new EndEffector(new EndEffectorIOSim());
          limelight = new Limelight(new LimelightIOReal(), drivetrain::getPose);
          break;

        case ROBOT_2023_RETIRED_ROBER:
          drivetrain =
              new Drivetrain(config, new GyroIOPigeon2(config), config.getSwerveModuleObjects());

          vision = new Vision(aprilTagLayout, drivetrain, config.getVisionModuleObjects());
          arm = new Arm(new ArmIO() {});
          elevator = new Elevator(new ElevatorIO() {});
          intake = new Intake(new IntakeIO() {});
          shooter = new Shooter(new ShooterIO() {});
          wrist = new Wrist(new WristIO() {});
          endEffector = new EndEffector(new EndEffectorIO() {});
          limelight = new Limelight(new LimelightIO() {}, drivetrain::getPose);
          break;

        case ROBOT_2024:
          drivetrain =
              new Drivetrain(config, new GyroIOPigeon2(config), config.getSwerveModuleObjects());

          vision = new Vision(aprilTagLayout, drivetrain, config.getVisionModuleObjects());
          arm = new Arm(new ArmIOReal());
          elevator = new Elevator(new ElevatorIOReal());
          intake = new Intake(new IntakeIOReal());
          shooter = new Shooter(new ShooterIOReal());
          wrist = new Wrist(new WristIOReal());
          endEffector = new EndEffector(new EndEffectorIOReal());
          limelight = new Limelight(new LimelightIOReal(), drivetrain::getPose);
          break;

        default:
          drivetrain =
              new Drivetrain(config, new GyroIO() {}, config.getReplaySwerveModuleObjects());
          vision = new Vision(aprilTagLayout, drivetrain, config.getReplayVisionModules());
          arm = new Arm(new ArmIO() {});
          elevator = new Elevator(new ElevatorIO() {});
          intake = new Intake(new IntakeIO() {});
          shooter = new Shooter(new ShooterIO() {});
          wrist = new Wrist(new WristIO() {});
          endEffector = new EndEffector(new EndEffectorIO() {});
          limelight = new Limelight(new LimelightIO() {}, drivetrain::getPose);
          break;
      }
    }

    drivetrain.setDefaultCommand(
        new DrivetrainDefaultTeleopDrive(
            drivetrain,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    intake.setDefaultCommand(new IntakeDefaultIdleRPM(intake));
    // Set up named commands for PathPlanner
    // NamedCommands.registerCommand(
    //     "Run Flywheel",
    //     Commands.startEnd(
    //         () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel));

    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up FF characterization routines
    // autoChooser.addOption(
    //     "Drive FF Characterization",
    //     new FeedForwardCharacterization(
    //         drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    // autoChooser.addOption(
    //     "Flywheel FF Characterization",
    //     new FeedForwardCharacterization(
    //         flywheel, flywheel::runCharacterizationVolts,
    // flywheel::getCharacterizationVelocity));

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
    controller.rightTrigger().whileTrue(new EjectGamepiece(intake));

    controller.y().onTrue(new ArmSetAngle(arm, Rotation2d.fromDegrees(90)));
    controller.b().onTrue(new ArmSetAngle(arm, Rotation2d.fromRadians(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.get();
    return new InstantCommand();
  }

  public double[] getCurrentDraws() {
    double[] current =
        new double[] {
          intake.getCurrentDraw(),
        };

    current = ArrayUtil.concatWithArrayCopy(current, drivetrain.getCurrentDrawAmps());
    return current;
  }

  public void updateVisualization() {
    SimpleMechanismVisualization.updateVisualization(arm.getAngle());
    SimpleMechanismVisualization.logMechanism();
  }
}
