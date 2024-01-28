// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.ScoringMode;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Drivetrain;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterShootMode extends Command {
  private Shooter shooter;
  private Drivetrain drive;
  private Translation2d speakerPose;
  private BooleanSupplier shootGamepiece = () -> false;
  private Double[] shootTimestamps = new Double[] {};
  private ArrayList<Double> shootTimestampsList;
  private Timer runTime = new Timer();

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
  public ShooterShootMode(Shooter shooter, Drivetrain drive, BooleanSupplier shootGamepiece) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.drive = drive;
    this.shootGamepiece = shootGamepiece;
    addRequirements(shooter);
    setName("ShooterShootMode");
  }

  public ShooterShootMode(Shooter shooter, Drivetrain drive, Double... shootTimestamps) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.drive = drive;
    this.shootTimestamps = shootTimestamps;
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
    runTime.start();

    for (Double timestamp : shootTimestamps) {
      shootTimestampsList.add(timestamp);
    }
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

    boolean shoot = shootGamepiece.getAsBoolean();

    for (Double timestamp : shootTimestamps) {
      if (timestamp <= runTime.get()) shoot = true;
    }

    if (shoot) {
      // run kicker
    }
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
