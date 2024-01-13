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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotMode.Mode;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.commands.Auto.AutoCommand;
import frc.robot.commands.Auto.Autos;
import frc.robot.commands.drive.DrivetrainDefaultTeleopDrive;
import frc.robot.configs.SimulatorRobotConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmReal;
import frc.robot.subsystems.arm.ArmSim;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberReal;
import frc.robot.subsystems.climber.ClimberSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeReal;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterReal;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionModule;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
  private final Climber climber;
  private final Intake intake;
  private final Shooter shooter;

  private final CommandXboxController controller = new CommandXboxController(0);

  private LoggedDashboardChooser<AutoCommand> autoChooser = new LoggedDashboardChooser<>("Auto Routine");

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
      climber = new Climber(new ClimberIO() {});
      intake = new Intake(new IntakeIO() {});
      shooter = new Shooter(new ShooterIO() {});

    } else { // REAL and SIM robots HERE
      switch (robotType) {
        case ROBOT_DEFAULT:
          drivetrain = null;
          vision = null;
          arm = null;
          climber = null;
          intake = null;
          shooter = null;
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

          vision = new Vision(aprilTagLayout, drivetrain, visionModules);
          arm = new Arm(new ArmSim());
          climber = new Climber(new ClimberSim());
          intake = new Intake(new IntakeSim());
          shooter = new Shooter(new ShooterSim());
          break;

        case ROBOT_2023_RETIRED_ROBER:
          drivetrain =
              new Drivetrain(config, new GyroIOPigeon2(config), config.getSwerveModuleObjects());

          vision = new Vision(aprilTagLayout, drivetrain, config.getVisionModuleObjects());
          arm = new Arm(new ArmIO() {});
          climber = new Climber(new ClimberIO() {});
          intake = new Intake(new IntakeIO() {});
          shooter = new Shooter(new ShooterIO() {});
          break;

        case ROBOT_2024:
          drivetrain =
              new Drivetrain(config, new GyroIOPigeon2(config), config.getSwerveModuleObjects());

          vision = new Vision(aprilTagLayout, drivetrain, config.getVisionModuleObjects());
          arm = new Arm(new ArmReal());
          climber = new Climber(new ClimberReal());
          intake = new Intake(new IntakeReal());
          shooter = new Shooter(new ShooterReal());
          break;

        default:
          drivetrain =
              new Drivetrain(config, new GyroIO() {}, config.getReplaySwerveModuleObjects());
          vision = new Vision(aprilTagLayout, drivetrain, config.getReplayVisionModules());
          arm = new Arm(new ArmIO() {});
          climber = new Climber(new ClimberIO() {});
          intake = new Intake(new IntakeIO() {});
          shooter = new Shooter(new ShooterIO() {});
          break;
      }
    }

    drivetrain.setDefaultCommand(
        new DrivetrainDefaultTeleopDrive(
            drivetrain,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

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
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public LoggedDashboardChooser<AutoCommand> getAutonomousChooser() {

    Autos autos = new Autos(drivetrain);

    for (int i = 0; i < autos.autoCommands().length; i++) {
      if (i == 0) {
        // Do nothing command must be first in list.
        autoChooser.addDefaultOption(autos.autoCommands()[i].name, autos.autoCommands()[i]);
      } else {
        autoChooser.addOption(autos.autoCommands()[i].name, autos.autoCommands()[i]);
      }
    }

    // TODO: add drive characterization command? maybe not necessary?
    // autoChooser.addOption(
    //   "Drive Characterization",
    //    new FeedForwardCharacterization(
    //        drivetrain,
    //        true,
    //        new FeedForwardCharacterizationData("drive"),
    //        drivetrain::runCharacterizationVolts,
    //        drivetrain::getCharacterizationVelocity));

    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());
    return autoChooser;
  }
  public void setPose(Pose2d pose){
    drivetrain.setPose(pose);
  }
}
