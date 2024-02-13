// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.autonomous;

// import com.choreo.lib.ChoreoTrajectory;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.lib.team2930.StateMachine;
// import frc.robot.DrivetrainWrapper;
// import frc.robot.configs.RobotConfig;
// import frc.robot.subsystems.endEffector.EndEffector;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.swerve.Drivetrain;

// public class AutoCommand extends StateMachine {
//   private Drivetrain drive;
//   private Shooter shooter;
//   private EndEffector endEffector;
//   private Intake intake;
//   private ChoreoHelper choreoHelper;
//   private DrivetrainWrapper driveWrapper;
//   private RobotConfig config;
//   private ChoreoTrajectory[] trajs;
//   private StateMachine stateMachine;
//   /** Creates a new AutoCommand. */
//   public AutoCommand(Drivetrain drive, Shooter shooter, EndEffector endEffector, Intake intake,
// RobotConfig config, ChoreoTrajectory... trajs) {
//     this.drive = drive;
//     this.shooter = shooter;
//     this.endEffector = endEffector;
//     this.intake = intake;
//     this.config = config;
//     this.trajs = trajs;
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   public void initialize() {
//     stateMachine.
//     choreoHelper = new ChoreoHelper(getName(), config.getAutoTranslationPidController(),
// config.getAutoThetaPidController(), drive.getPoseEstimatorPose());
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   public void execute() {
//     driveWrapper.setVelocity(choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(),
// 0));
//   }

//   // Called once the command ends or is interrupted.
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   public boolean isFinished() {
//     return false;
//   }
// }