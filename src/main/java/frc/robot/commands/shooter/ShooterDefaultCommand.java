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
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Drivetrain;

public class ShooterDefaultCommand extends Command {
  private Shooter shooter;
  private Drivetrain drive;
  private Translation2d speakerPose;

  private LoggedTunableNumber kp = new LoggedTunableNumber("ShooterDefaultCommand/pitchKp", 3.0);
  private LoggedTunableNumber ki = new LoggedTunableNumber("ShooterDefaultCommand/pitchKi", 0.0);
  private LoggedTunableNumber kd = new LoggedTunableNumber("ShooterDefaultCommand/pitchKd", 0.0);
  private LoggedTunableNumber tolerance =
      new LoggedTunableNumber("ShooterDefaultCommand/pitchKd", 1.0);
  private PIDController shooterPitchPID = new PIDController(kp.get(), ki.get(), kd.get());

  private Rotation2d speakerHeading;
  private double shooterPitchVelCorrection = 0.0;

  /** Creates a new ShooterDefaultCommand. */
  public ShooterDefaultCommand(Shooter shooter, Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.drive = drive;
    addRequirements(shooter);
    setName("ShooterDefaultCommand");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speakerPose =
        new Translation2d(
            DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                ? 0.23826955258846283
                : 16.281435012817383,
            5.498747638702393);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterPitchPID.setTolerance(Math.toRadians(tolerance.get()));
    if (RobotState.getInstance().getShootMode().equals(RobotState.ShootMode.SPEAKER)) {
      double distToSpeaker =
          Math.hypot(
              drive.getRawOdometryPose().getX() - speakerPose.getX(),
              drive.getRawOdometryPose().getY() - speakerPose.getY());
      shooter.setRPM(
          DriverStation.isAutonomous()
              ? Constants.ShooterConstants.SHOOTING_RPM
              : Constants.ShooterConstants.PREP_RPM);
      speakerHeading =
          new Rotation2d(
              drive.getRawOdometryPose().getX() - speakerPose.getX(),
              drive.getRawOdometryPose().getY() - speakerPose.getY());
      shooterPitchVelCorrection =
          Constants.ShooterConstants.PITCH_VEL_RAD_PER_SEC(
              new Translation2d(
                      drive.getFieldRelativeVelocities().getX(),
                      drive.getFieldRelativeVelocities().getY())
                  .rotateBy(speakerHeading)
                  .getY(),
              distToSpeaker);
      shooter.setPitchAngularVel(
          shooterPitchPID.calculate(
                  shooter.getPitch().getRadians(),
                  Constants.ShooterConstants.DISTANCE_TO_SHOOTING_PITCH(distToSpeaker).getRadians())
              + shooterPitchVelCorrection);
    } else {
      shooter.setRPM(0.0);
      shooter.setPitchAngularVel(
          shooterPitchPID.calculate(
              shooter.getPitch().getRadians(),
              Constants.ShooterConstants.SHOOTER_STOW_PITCH.getRadians()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
