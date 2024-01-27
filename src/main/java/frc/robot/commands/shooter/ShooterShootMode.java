// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.ScoringMode;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Drivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterShootMode extends Command {
  private Shooter shooter;
  private Drivetrain drive;
  private Translation2d speakerPose;
  private DoubleSupplier shooterRPM;

  // private LoggedTunableNumber kp = new LoggedTunableNumber("ShooterDefaultCommand/pitchKp",
  // 10.0);
  // private LoggedTunableNumber ki = new LoggedTunableNumber("ShooterDefaultCommand/pitchKi", 0.0);
  // private LoggedTunableNumber kd = new LoggedTunableNumber("ShooterDefaultCommand/pitchKd", 0.0);
  // private LoggedTunableNumber tolerance =
  //     new LoggedTunableNumber("ShooterDefaultCommand/pitchKd", 1.0);
  // private PIDController shooterPitchPID = new PIDController(kp.get(), ki.get(), kd.get());

  // private Rotation2d speakerHeading;
  // private double shooterPitchVelCorrection = 0.0;
  private BooleanSupplier shootSupplier;

  /** Creates a new ShooterDefaultCommand. */
  public ShooterShootMode(
      Shooter shooter,
      EndEffector endEffector,
      Drivetrain drive,
      BooleanSupplier shootSupplier,
      DoubleSupplier shooterRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.drive = drive;
    this.shootSupplier = shootSupplier;
    this.shooterRPM = shooterRPM;
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
    Pose2d futurePose =
        drive.getFutureEstimatedPose(shooter.getPivotPIDLatency(), "ShooterShootMode");

    double horizDist =
        Math.hypot(futurePose.getX() - speakerPose.getX(), futurePose.getY() - speakerPose.getY());

    double dist = Math.hypot(horizDist, Constants.FieldConstants.SPEAKER_HEIGHT_METERS);

    double shooterTangentialSpeed =
        shooterRPM.getAsDouble()
            * 60.0
            * Math.PI
            * Constants.ShooterConstants.Launcher.WHEEL_DIAMETER_METERS;

    double shotTime = dist / shooterTangentialSpeed;

    Translation2d virtualSpeakerTranslation =
        speakerPose.minus(
            new Translation2d(drive.getFieldRelativeVelocities().getX() * shotTime, 0.0));

    shooter.setPercentOut(Constants.ShooterConstants.SHOOTING_RPM);

    Translation2d shooterBaseTranslation =
        futurePose.transformBy(Constants.ShooterConstants.SHOOTER_OFFSET_METERS).getTranslation();

    double distToSpeaker =
        Math.hypot(
            shooterBaseTranslation.getX() - speakerPose.getX(),
            shooterBaseTranslation.getY() - speakerPose.getY());

    Logger.recordOutput("ShooterShootMode/futurePose", futurePose);

    Logger.recordOutput(
        "ShooterShootMode/futureShooterBasePose",
        new Pose2d(shooterBaseTranslation, futurePose.getRotation()));

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
