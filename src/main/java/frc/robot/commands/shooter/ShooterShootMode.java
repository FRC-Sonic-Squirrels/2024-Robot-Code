// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.ScoringMode;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Drivetrain;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterShootMode extends Command {
  private Shooter shooter;
  private Drivetrain drive;
  private Translation2d speakerPose;

  private LoggedTunableNumber kp = new LoggedTunableNumber("ShooterDefaultCommand/pitchKp", 10.0);
  private LoggedTunableNumber ki = new LoggedTunableNumber("ShooterDefaultCommand/pitchKi", 0.0);
  private LoggedTunableNumber kd = new LoggedTunableNumber("ShooterDefaultCommand/pitchKd", 0.0);
  private LoggedTunableNumber tolerance =
      new LoggedTunableNumber("ShooterDefaultCommand/pitchKd", 1.0);
  private PIDController shooterPitchPID = new PIDController(kp.get(), ki.get(), kd.get());

  private Rotation2d speakerHeading;
  private double shooterPitchVelCorrection = 0.0;
  private BooleanSupplier shootSupplier;

  /** Creates a new ShooterDefaultCommand. */
  public ShooterShootMode(
      Shooter shooter, EndEffector endEffector, Drivetrain drive, BooleanSupplier shootSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.drive = drive;
    this.shootSupplier = shootSupplier;
    addRequirements(shooter);
    setName("ShooterShootMode");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().isPresent()) {
      speakerPose =
          new Translation2d(
              DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                  ? 0.23826955258846283
                  : 16.281435012817383,
              5.498747638702393);
    } else {
      speakerPose = new Translation2d(0.23826955258846283, 5.498747638702393);
    }
  }

  // TODO: find everywhere that rawodometry pose is used and evaluate whether its needed

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterPitchPID.setTolerance(Math.toRadians(tolerance.get()));

    shooter.setLauncherClosedLoopConstants(10.0, 0, 0);
    double distToSpeaker =
        Math.hypot(
            drive.getPoseEstimatorPose().getX() - speakerPose.getX(),
            drive.getPoseEstimatorPose().getY() - speakerPose.getY());

    if (shootSupplier.getAsBoolean()) {
      shooter.setRPM(Constants.ShooterConstants.SHOOTING_RPM);
      if (shooter.launcherIsAtTargetVel()) {
        // TODO: logic for end effector running. Also check if arm is in correct position
      }
    } else {
      shooter.setRPM(
          DriverStation.isAutonomous()
              ? Constants.ShooterConstants.SHOOTING_RPM
              : Constants.ShooterConstants.PREP_RPM);
    }
    speakerHeading =
        new Rotation2d(
            drive.getPoseEstimatorPose().getX() - speakerPose.getX(),
            drive.getPoseEstimatorPose().getY() - speakerPose.getY());
    double linearVelSpeaker =
        new Translation2d(
                drive.getFieldRelativeVelocities().getX(),
                drive.getFieldRelativeVelocities().getY())
            .rotateBy(speakerHeading)
            .getX();
    shooterPitchVelCorrection =
        Constants.ShooterConstants.Pitch.PITCH_VEL_RAD_PER_SEC(linearVelSpeaker, distToSpeaker);

    double targetAngle =
        Constants.ShooterConstants.Pitch.DISTANCE_TO_SHOOTING_PITCH(distToSpeaker).getRadians();
    double vel =
        shooterPitchPID.calculate(shooter.getPitch().getRadians(), targetAngle)
            + shooterPitchVelCorrection;
    shooter.setPitchAngularVel(vel);

    Logger.recordOutput("ShooterShootMode/velCorrection", shooterPitchVelCorrection);
    Logger.recordOutput("ShooterShootMode/vel", vel);
    Logger.recordOutput("ShooterShootMode/targetAngleDegrees", Math.toDegrees(targetAngle));
    Logger.recordOutput("ShooterShootMode/linearVelSpeaker", linearVelSpeaker);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotState.getInstance().getScoringMode().equals(ScoringMode.SPEAKER);
  }
}
