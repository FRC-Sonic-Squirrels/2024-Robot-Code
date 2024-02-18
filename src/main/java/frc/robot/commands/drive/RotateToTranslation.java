// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.PIDTargetMeasurement;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.Drivetrain;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RotateToTranslation extends Command {
  private Supplier<Translation2d> target;
  private Drivetrain drive;
  private Supplier<Boolean> endCondition;

  private Rotation2d robotRotationOffset;

  private final LoggedTunableNumber rotationKp =
      new LoggedTunableNumber("RotateToTranslation/rotationKp", 4.9);
  private final LoggedTunableNumber rotationKd =
      new LoggedTunableNumber("RotateToTranslation/rotationKd", 0.0);

  private final PIDController rotationController;

  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier omegaSupplier;
  private final double driverInfluencePercent;

  private double rotVelCorrection = 0;

  public static final double DEADBAND = 0.1;

  // Creates a new flat moving average filter
  // Average will be taken over the last 20 samples
  private LinearFilter pidLatencyfilter = LinearFilter.movingAverage(20);

  private ArrayList<PIDTargetMeasurement> targetMeasurements =
      new ArrayList<PIDTargetMeasurement>();

  private double pidLatency = 0.0;

  /**
   * Creates a new RotateToTranslation.
   *
   * @param translationXSupplier controller horizontal translation output
   * @param translationYSupplier controller vertical translation output
   * @param target pose to target
   * @param drive drivetrain subsystem
   * @return Command to lock rotation in direction of target gamepiece
   */
  public RotateToTranslation(
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      Supplier<Translation2d> target,
      Drivetrain drive,
      Supplier<Boolean> endCondition) {
    this(
        translationXSupplier,
        translationYSupplier,
        target,
        drive,
        endCondition,
        new Rotation2d(0.0));
  }

  /**
   * Creates a new RotateToTranslation.
   *
   * @param translationXSupplier controller horizontal translation output
   * @param translationYSupplier controller vertical translation output
   * @param target pose to target
   * @param drive drivetrain subsystem
   * @param endCondition when this command should end
   * @param robotRotationOffset rotation of robot you want facing taget
   * @return Command to lock rotation in direction of target
   */
  public RotateToTranslation(
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      Supplier<Translation2d> target,
      Drivetrain drive,
      Supplier<Boolean> endCondition,
      Rotation2d robotRotationOffset) {
    this(
        translationXSupplier,
        translationYSupplier,
        target,
        drive,
        endCondition,
        robotRotationOffset,
        () -> 0.0,
        0.0);
  }

  /**
   * Creates a new RotateToTranslation.
   *
   * @param translationXSupplier controller horizontal translation output
   * @param translationYSupplier controller vertical translation output
   * @param target pose to target
   * @param drive drivetrain subsystem
   * @param endCondition when this command should end
   * @param robotRotationOffset rotation of robot you want facing target
   * @param omegaSupplier controller rotation output
   * @return Command to lock rotation in direction of target
   */
  public RotateToTranslation(
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      Supplier<Translation2d> target,
      Drivetrain drive,
      Supplier<Boolean> endCondition,
      Rotation2d robotRotationOffset,
      DoubleSupplier omegaSupplier) {
    this(
        translationXSupplier,
        translationYSupplier,
        target,
        drive,
        endCondition,
        robotRotationOffset,
        omegaSupplier,
        0.3);
  }

  // TODO: need to add driver rotation so driver can affect rotation when objects are not seen/fight
  // a bit
  // TODO: behavior is decent but pid needs to be tuned

  // TODO: maybe we want to instead make a driveAndRotateCommand(Supplier<Rotation2d>
  // targetRotation)
  // that way it is generic and not tied just to limelight?
  //
  /**
   * Creates a new RotateToTranslation.
   *
   * @param translationXSupplier controller horizontal translation output
   * @param translationYSupplier controller vertical translation output
   * @param target pose to target
   * @param drive drivetrain subsystem
   * @param endCondition when this command should end
   * @param robotRotationOffset rotation of robot you want facing target
   * @param omegaSupplier controller rotation output
   * @param driverInfluencePercent 0.0 (rotation is fully based on command) -> 1.0 (when
   *     omegaSupplier is at 1 or -1, rotation is fully based on driver input)
   * @return Command to lock rotation in direction of target
   */
  public RotateToTranslation(
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      Supplier<Translation2d> target,
      Drivetrain drive,
      Supplier<Boolean> endCondition,
      Rotation2d robotRotationOffset,
      DoubleSupplier omegaSupplier,
      Double driverInfluencePercent) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.target = target;
    this.drive = drive;
    this.endCondition = endCondition;
    this.robotRotationOffset = robotRotationOffset;
    this.omegaSupplier = omegaSupplier;
    this.driverInfluencePercent = driverInfluencePercent;

    this.rotationController = new PIDController(rotationKp.get(), 0, rotationKd.get());
    addRequirements(drive);
    setName("RotateToTranslation");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(Math.toRadians(5));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // TODO: add some kind of controller utilizties to make this easier and not have to copy code
    // from
    // DrivetrainDeafultTeleopDrive

    Pose2d futurePose = drive.getFutureEstimatedPose(pidLatency, "RotateToTranslation");

    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble()),
            DEADBAND);
    Rotation2d linearDirection =
        new Rotation2d(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble());
    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega = Math.copySign(omega * omega, omega);

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    var currentRot = drive.getRotation();

    Rotation2d heading = GeometryUtil.getHeading(futurePose.getTranslation(), target.get());

    var goalRot = heading.plus(robotRotationOffset);

    var driverRotVel = omega * drive.getMaxAngularSpeedRadPerSec();

    var rotationalEffort =
        (rotationController.calculate(currentRot.getRadians(), goalRot.getRadians())
                    + rotVelCorrection)
                * (1 - Math.abs(omegaSupplier.getAsDouble() * driverInfluencePercent))
            + driverRotVel * driverInfluencePercent;

    rotationalEffort =
        Math.copySign(
            Math.min(Math.abs(rotationalEffort), drive.getMaxAngularSpeedRadPerSec()),
            rotationalEffort);

    if (rotationController.atSetpoint()) {
      rotationalEffort = driverRotVel * driverInfluencePercent;
    }

    var xVel = linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec();
    var yVel = linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec();

    rotVelCorrection =
        Math.hypot(xVel, yVel)
            * Math.cos(heading.getRadians() - new Rotation2d(xVel, yVel).getRadians() - Math.PI / 2)
            / Math.hypot(
                futurePose.getX() - target.get().getX(), futurePose.getY() - target.get().getY());

    targetMeasurements.add(
        new PIDTargetMeasurement(
            Timer.getFPGATimestamp(),
            goalRot,
            drive.getRotation().getRadians() <= goalRot.getRadians()));
    for (int index = 0; index < targetMeasurements.size(); index++) {
      PIDTargetMeasurement measurement = targetMeasurements.get(index);
      if ((measurement.upDirection
              && drive.getRotation().getRadians() >= measurement.targetRot.getRadians())
          || (!measurement.upDirection
              && drive.getRotation().getRadians() <= measurement.targetRot.getRadians())) {
        pidLatency = pidLatencyfilter.calculate(Timer.getFPGATimestamp() - measurement.timestamp);
        targetMeasurements.remove(index);
      } else if (Timer.getFPGATimestamp() - measurement.timestamp >= 1.0) {
        targetMeasurements.remove(index);
      }
    }
    Logger.recordOutput("RotateToTranslation/PIDLatency", pidLatency);

    var robotChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotationalEffort, drive.getRotation());
    drive.runVelocity(robotChassisSpeeds, false);

    // TODO: remove most of these once we are happy with the command
    Logger.recordOutput("RotateToTranslation/RotationalEffort", rotationalEffort);
    Logger.recordOutput(
        "RotateToTranslation/rotationalErrorDegrees",
        Math.toDegrees(rotationController.getPositionError()));
    Logger.recordOutput("RotateToTranslation/desiredLinearVelocity", linearVelocity);
    Logger.recordOutput("RotateToTranslation/rotationCorrection", rotVelCorrection);
    Logger.recordOutput(
        "RotateToTranslation/robotRotationDegrees", drive.getRotation().getDegrees());
    Logger.recordOutput("RotateToTranslation/targetRotationDegrees", goalRot.getDegrees());
    Logger.recordOutput(
        "RotateToTranslation/optimalRotationDegrees",
        GeometryUtil.getHeading(drive.getPoseEstimatorPose().getTranslation(), target.get())
            .plus(robotRotationOffset)
            .getDegrees());
    Logger.recordOutput("RotateToTranslation/atSetpoint", rotationController.atSetpoint());

    if (rotationKp.hasChanged(hashCode()) || rotationKd.hasChanged(hashCode())) {
      rotationController.setP(rotationKp.get());
      rotationController.setD(rotationKd.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0, 0, 0), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCondition.get();
  }
}
