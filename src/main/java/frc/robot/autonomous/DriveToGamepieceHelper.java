package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

  private static final LoggedTunableNumber allowedRotationalErrorDegrees =
      group.build("allowedRotationalErrorDegrees", 10);

  private static final LoggedTunableNumber advancedMode =
      group.build("advancedMode/doAdvancedMotion", 1);

  private final PIDController xController = new PIDController(0, 0, 0);
  private final PIDController yController = new PIDController(0, 0, 0);
  private final PIDController rotController = new PIDController(0, 0, 0);

  private Rotation2d gamepieceDirection = Constants.zeroRotation2d;

  public DriveToGamepieceHelper() {
    xController.reset();
    xController.setP(kP.get());
    xController.setI(kI.get());
    xController.setD(kD.get());
    xController.setTolerance(0.01);

    yController.reset();
    yController.setP(kP.get());
    yController.setI(kI.get());
    yController.setD(kD.get());
    yController.setTolerance(0.01);

    rotController.reset();
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    rotController.setP(rotationKp.get());
    rotController.setTolerance(2);
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
