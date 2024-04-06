package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;

public class DriveToGamepieceHelper {
  private static final String ROOT_TABLE = "DriveToGamepiece";

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Decimal log_rotationalErrorDegrees =
      logGroup.buildDecimal("rotationalError");
  private static final LoggerEntry.Decimal log_xVel = logGroup.buildDecimal("xVel");
  private static final LoggerEntry.Decimal log_yVel = logGroup.buildDecimal("yVel");
  private static final LoggerEntry.Decimal log_rotVel = logGroup.buildDecimal("rotVel");
  private static final LoggerEntry.Bool log_isAtTarget = logGroup.buildBoolean("isAtTarget");

  private static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);

  private static final LoggedTunableNumber kP = group.build("kP", 2.0);
  private static final LoggedTunableNumber kI = group.build("kI", 0.0);
  private static final LoggedTunableNumber kD = group.build("kD", 0.0);

  private static final LoggedTunableNumber rotationKp = group.build("rotationKp", 4.8);

  private static final LoggedTunableNumber rotationMaxAcceleration =
      group.build("rotationMaxAcceleration", 720.0);

  private static final LoggedTunableNumber rotationMaxVelocity =
      group.build("rotationMaxVelocity", 360.0);

  private static final LoggedTunableNumber allowedRotationalErrorDegrees =
      group.build("allowedRotationalErrorDegrees", 10);

  private static final LoggedTunableNumber advancedMode =
      group.build("advancedMode/doAdvancedMotion", 1);

  private static final LoggedTunableNumber maxSpeed = group.build("advancedMode/maxSpeed", 3);

  private final PIDController xController = new PIDController(0, 0, 0);
  private final PIDController yController = new PIDController(0, 0, 0);
  private final ProfiledPIDController rotController =
      new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));

  private Rotation2d gamepieceDirection = Constants.zeroRotation2d;

  public DriveToGamepieceHelper(Pose2d currentPose, Pose2d currentVel) {
    xController.reset();
    xController.setP(kP.get());
    xController.setI(kI.get());
    xController.setD(kD.get());
    xController.setTolerance(0.05);

    yController.reset();
    yController.setP(kP.get());
    yController.setI(kI.get());
    yController.setD(kD.get());
    yController.setTolerance(0.05);

    rotController.reset(
        currentPose.getRotation().getRadians(), currentVel.getRotation().getRadians());
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    rotController.setP(rotationKp.get());
    rotController.setConstraints(
        new Constraints(
            Math.toRadians(rotationMaxVelocity.get()),
            Math.toRadians(rotationMaxAcceleration.get())));
    rotController.setTolerance(Math.toRadians(2));
  }

  public ChassisSpeeds calculateChassisSpeeds(Translation2d targetGamepiece, Pose2d pose) {
    if (targetGamepiece == null) return null;
    double rotationalErrorDegrees;
    double xVel;
    double yVel;
    double rotVel;

    if (GeometryUtil.getDist(pose.getTranslation(), targetGamepiece) > 0.2) {
      gamepieceDirection =
          new Rotation2d(
              targetGamepiece.getX() - pose.getX(), targetGamepiece.getY() - pose.getY());
    }

    rotationalErrorDegrees = gamepieceDirection.minus(pose.getRotation()).getDegrees();
    rotationalErrorDegrees = GeometryUtil.optimizeRotationInDegrees(rotationalErrorDegrees);
    rotationalErrorDegrees = Math.abs(rotationalErrorDegrees);

    double allowedRotationalErrorDegreesValue = allowedRotationalErrorDegrees.get();
    if (advancedMode.get() == 0) {
      xVel =
          rotationalErrorDegrees < allowedRotationalErrorDegreesValue
              ? xController.calculate(pose.getX(), targetGamepiece.getX())
              : 0.0;
      yVel =
          rotationalErrorDegrees < allowedRotationalErrorDegreesValue
              ? yController.calculate(pose.getY(), targetGamepiece.getY())
              : 0.0;
    } else {
      xVel = xController.calculate(pose.getX(), targetGamepiece.getX());
      yVel = yController.calculate(pose.getY(), targetGamepiece.getY());

      if (rotationalErrorDegrees > allowedRotationalErrorDegreesValue) {
        double errorScaling =
            Math.min(allowedRotationalErrorDegreesValue / rotationalErrorDegrees, 1.0);
        xVel *= errorScaling;
        yVel *= errorScaling;
      }
    }

    Translation2d vel = new Translation2d(xVel, yVel);

    double mag = vel.getNorm();

    if (mag > maxSpeed.get()) {
      vel = vel.div(mag).times(maxSpeed.get());
    }

    xVel = vel.getX();
    yVel = vel.getY();

    rotVel =
        rotController.calculate(pose.getRotation().getRadians(), gamepieceDirection.getRadians());

    log_rotationalErrorDegrees.info(rotationalErrorDegrees);
    log_xVel.info(xVel);
    log_yVel.info(yVel);
    log_rotVel.info(rotVel);
    log_isAtTarget.info(
        xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint());

    return ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, pose.getRotation());
  }

  public boolean isAtTarget() {
    return xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint();
  }
}
