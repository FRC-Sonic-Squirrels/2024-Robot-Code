// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.limelight.ProcessedGamepieceData;
import frc.robot.subsystems.swerve.Drivetrain;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RotateToGamepiece extends Command {
  private Supplier<ProcessedGamepieceData> targetGamepiece;
  private Drivetrain drive;
  private Supplier<Boolean> gamepieceIntaked;

  private Rotation2d robotRotationOffset;

  private final LoggedTunableNumber rotationKp =
      new LoggedTunableNumber("RotateToGamepiece/rotationKp", 4.9);
  private final LoggedTunableNumber rotationKd =
      new LoggedTunableNumber("RotateToGamepiece/rotationKd", 0.0);

  private final PIDController rotationController;

  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier omegaSupplier;
  private final double driverInfluencePercent;

  private double rotVelCorrection = 0;

  public static final double DEADBAND = 0.1;

  /**
   * Creates a new RotateToGamepiece.
   *
   * @param translationXSupplier controller horizontal translation output
   * @param translationYSupplier controller vertical translation output
   * @param targetGamepiece gamepiece to target
   * @param drive drivetrain subsystem
   * @return Command to lock rotation in direction of target gamepiece
   */
  public RotateToGamepiece(
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      Supplier<ProcessedGamepieceData> targetGamepiece,
      Drivetrain drive,
      Supplier<Boolean> gamepieceIntaked) {
    this(
        translationXSupplier,
        translationYSupplier,
        targetGamepiece,
        drive,
        gamepieceIntaked,
        new Rotation2d(0.0));
  }

  /**
   * Creates a new RotateToGamepiece.
   *
   * @param translationXSupplier controller horizontal translation output
   * @param translationYSupplier controller vertical translation output
   * @param targetGamepiece gamepiece to target
   * @param drive drivetrain subsystem
   * @param robotRotationOffset rotation of robot you want facing gamepiece
   * @return Command to lock rotation in direction of target gamepiece
   */
  public RotateToGamepiece(
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      Supplier<ProcessedGamepieceData> targetGamepiece,
      Drivetrain drive,
      Supplier<Boolean> gamepieceIntaked,
      Rotation2d robotRotationOffset) {
    this(
        translationXSupplier,
        translationYSupplier,
        targetGamepiece,
        drive,
        gamepieceIntaked,
        robotRotationOffset,
        () -> 0.0,
        0.0);
  }

  /**
   * Creates a new RotateToGamepiece.
   *
   * @param translationXSupplier controller horizontal translation output
   * @param translationYSupplier controller vertical translation output
   * @param targetGamepiece gamepiece to target
   * @param drive drivetrain subsystem
   * @param robotRotationOffset rotation of robot you want facing gamepiece
   * @param omegaSupplier controller rotation output
   * @return Command to lock rotation in direction of target gamepiece
   */
  public RotateToGamepiece(
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      Supplier<ProcessedGamepieceData> targetGamepiece,
      Drivetrain drive,
      Supplier<Boolean> gamepieceIntaked,
      Rotation2d robotRotationOffset,
      DoubleSupplier omegaSupplier) {
    this(
        translationXSupplier,
        translationYSupplier,
        targetGamepiece,
        drive,
        gamepieceIntaked,
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
   * Creates a new RotateToGamepiece.
   *
   * @param translationXSupplier controller horizontal translation output
   * @param translationYSupplier controller vertical translation output
   * @param targetGamepiece gamepiece to target
   * @param drive drivetrain subsystem
   * @param robotRotationOffset rotation of robot you want facing gamepiece
   * @param omegaSupplier controller rotation output
   * @param driverInfluencePercent 0.0 (rotation is fully based on command) -> 1.0 (when
   *     omegaSupplier is at 1 or -1, rotation is fully based on driver input)
   * @return Command to lock rotation in direction of target gamepiece
   */
  public RotateToGamepiece(
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      Supplier<ProcessedGamepieceData> targetGamepiece,
      Drivetrain drive,
      Supplier<Boolean> gamepieceIntaked,
      Rotation2d robotRotationOffset,
      DoubleSupplier omegaSupplier,
      Double driverInfluencePercent) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.targetGamepiece = targetGamepiece;
    this.drive = drive;
    this.gamepieceIntaked = gamepieceIntaked;
    this.robotRotationOffset = robotRotationOffset;
    this.omegaSupplier = omegaSupplier;
    this.driverInfluencePercent = driverInfluencePercent;

    this.rotationController = new PIDController(rotationKp.get(), 0, rotationKd.get());
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(Units.degreesToRadians(5));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // TODO: add some kind of controller utilizties to make this easier and not have to copy code
    // from
    // DrivetrainDeafultTeleopDrive

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

    var goalRot = targetGamepiece.get().targetYaw.plus(robotRotationOffset);

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

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotationalEffort, drive.getRotation()));

    rotVelCorrection =
        Math.hypot(xVel, yVel)
            * Math.cos(
                targetGamepiece.get().targetYaw.getRadians()
                    - new Rotation2d(xVel, yVel).getRadians()
                    - Math.PI / 2)
            / targetGamepiece.get().distance;

    // TODO: remove most of these once we are happy with the command
    Logger.recordOutput("RotateToGamepiece/RotationalEffort", rotationalEffort);
    Logger.recordOutput("ActiveCommands/RotateToGamepiece", true);
    Logger.recordOutput(
        "RotateToGamepiece/rotationalErrorDegrees",
        Units.radiansToDegrees(rotationController.getPositionError()));
    Logger.recordOutput("RotateToGamepiece/desiredLinearVelocity", linearVelocity);
    Logger.recordOutput("RotateToGamepiece/rotationCorrection", rotVelCorrection);
    Logger.recordOutput("RotateToGamepiece/robotRotationDegrees", drive.getRotation().getDegrees());
    Logger.recordOutput("RotateToGamepiece/atSetpoint", rotationController.atSetpoint());

    if (rotationKp.hasChanged(hashCode()) || rotationKd.hasChanged(hashCode())) {
      rotationController.setP(rotationKp.get());
      rotationController.setD(rotationKd.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0, 0, 0));
    Logger.recordOutput("ActiveCommands/RotateToGamepiece", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gamepieceIntaked.get();
  }
}
