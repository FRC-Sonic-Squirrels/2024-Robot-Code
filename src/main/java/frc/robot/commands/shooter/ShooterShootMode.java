// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.ScoringMode;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Drivetrain;
import org.littletonrobotics.junction.Logger;

public class ShooterShootMode extends Command {
  private Shooter shooter;
  private Drivetrain drive;
  private Translation2d speakerPose;

  // private LoggedTunableNumber kp = new LoggedTunableNumber("ShooterDefaultCommand/pitchKp",
  // 10.0);
  // private LoggedTunableNumber ki = new LoggedTunableNumber("ShooterDefaultCommand/pitchKi", 0.0);
  // private LoggedTunableNumber kd = new LoggedTunableNumber("ShooterDefaultCommand/pitchKd", 0.0);
  // private LoggedTunableNumber tolerance =
  //     new LoggedTunableNumber("ShooterDefaultCommand/pitchKd", 1.0);
  // private PIDController shooterPitchPID = new PIDController(kp.get(), ki.get(), kd.get());

  // private Rotation2d speakerHeading;
  // private double shooterPitchVelCorrection = 0.0;

  /** Creates a new ShooterDefaultCommand. */
  public ShooterShootMode(Shooter shooter, Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.drive = drive;
    addRequirements(shooter);
    setName("ShooterShootMode");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speakerPose = Constants.FieldConstants.getSpeakerTranslation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d usedRobotPose =
        drive.getFutureEstimatedPose(shooter.getPivotPIDLatency(), "ShooterShootMode");

    double horizDist =
        Math.hypot(
            usedRobotPose.getX() - speakerPose.getX(), usedRobotPose.getY() - speakerPose.getY());

    double dist = Math.hypot(horizDist, Constants.FieldConstants.SPEAKER_HEIGHT_METERS);

    double shooterTangentialSpeed =
        shooter.getRPM()
            * 60.0
            * Math.PI
            * Constants.ShooterConstants.Launcher.WHEEL_DIAMETER_METERS;

    double shotTime = dist / shooterTangentialSpeed;

    Translation2d virtualSpeakerTranslation =
        speakerPose.minus(
            new Translation2d(drive.getFieldRelativeVelocities().getX() * shotTime, 0.0));

    shooter.setPercentOut(Constants.ShooterConstants.SHOOTING_RPM);

    Translation2d shooterBaseTranslation =
        usedRobotPose
            .transformBy(Constants.ShooterConstants.SHOOTER_OFFSET_METERS)
            .getTranslation();

    double distToSpeaker =
        Math.hypot(
            shooterBaseTranslation.getX() - virtualSpeakerTranslation.getX(),
            shooterBaseTranslation.getY() - virtualSpeakerTranslation.getY());

    Logger.recordOutput("ShooterShootMode/usedRobotPose", usedRobotPose);

    Logger.recordOutput(
        "ShooterShootMode/futureShooterBasePose",
        new Pose2d(shooterBaseTranslation, usedRobotPose.getRotation()));

    // VELOCITY CONTROL

    // shooterPitchPID.setTolerance(Math.toRadians(tolerance.get()));

    // if (shootSupplier.getAsBoolean()) {
    //
    //   // if (shooter.launcherIsAtTargetVel()) {
    //   // TODO: logic for end effector running. Also check if arm is in correct position
    //   // }
    // speakerHeading =
    //     new Rotation2d(
    //         drive.getPoseEstimatorPose().getX() - speakerPose.getX(),
    //         drive.getPoseEstimatorPose().getY() - speakerPose.getY());
    // double linearVelSpeaker =
    //     new Translation2d(
    //             drive.getFieldRelativeVelocities().getX(),
    //             drive.getFieldRelativeVelocities().getY())
    //         .rotateBy(speakerHeading)
    //         .getX();
    // shooterPitchVelCorrection =
    //     Constants.ShooterConstants.Pivot.PITCH_VEL_RAD_PER_SEC(linearVelSpeaker, distToSpeaker);

    // double targetAngle =
    //     Constants.ShooterConstants.Pivot.DISTANCE_TO_SHOOTING_PITCH(distToSpeaker).getRadians();
    // double vel =
    //     shooterPitchPID.calculate(shooter.getPitch().getRadians(), targetAngle)
    //         + shooterPitchVelCorrection;
    // shooter.setPitchAngularVel(vel);

    // POSITIONAL CONTROL

    shooter.setPivotPosition(
        Constants.ShooterConstants.Pivot.DISTANCE_TO_SHOOTING_PITCH(distToSpeaker));

    // Logger.recordOutput("ShooterShootMode/velCorrection", shooterPitchVelCorrection);
    // Logger.recordOutput("ShooterShootMode/vel", vel);
    // Logger.recordOutput("ShooterShootMode/targetAngleDegrees", Math.toDegrees(targetAngle));
    // Logger.recordOutput("ShooterShootMode/linearVelSpeaker", linearVelSpeaker);

    Translation2d currentShooterTranslation =
        drive
            .getPoseEstimatorPose()
            .transformBy(Constants.ShooterConstants.SHOOTER_OFFSET_METERS)
            .getTranslation();

    Logger.recordOutput(
        "ShooterShootMode/optimalAngle",
        Constants.ShooterConstants.Pivot.DISTANCE_TO_SHOOTING_PITCH(
                Math.hypot(
                    currentShooterTranslation.getX() - speakerPose.getX(),
                    currentShooterTranslation.getY() - speakerPose.getY()))
            .getDegrees());
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
