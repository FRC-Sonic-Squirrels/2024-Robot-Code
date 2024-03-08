package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class DriveToGamepieceHelper {
  private PIDController xController;
  private PIDController yController;
  private PIDController rotController;

  private TunableNumberGroup group = new TunableNumberGroup("DriveToGamepiece");

  private LoggedTunableNumber kP = group.build("kX", 1.0);
  private LoggedTunableNumber kI = group.build("kX", 0.0);
  private LoggedTunableNumber kD = group.build("kX", 0.0);

  private LoggedTunableNumber rotationKp = group.build("rotationKp", 0.4);

  private LoggedTunableNumber xKpSourceArea = group.build("xKpSourceArea", 0.1);
  private LoggedTunableNumber yKpSourceArea = group.build("yKpSourceArea", 0.1);

  private LoggedTunableNumber allowedRotationalErrorDegrees =
      group.build("allowedRotationalErrorDegrees", 20);
  private LoggedTunableNumber advancedMode = group.build("advancedMode/doAdvancedMotion", 1);

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

  private boolean isInSourceArea(Translation2d pose) {
    return (pose.getX() >= 14.26539134979248 || pose.getX() <= 2.154930830001831)
        && pose.getY() <= 1.803399920463562;
  }

  private boolean isInBlueSourceArea(Translation2d pose) {
    return pose.getX() >= 14.26539134979248 && pose.getY() <= 1.803399920463562;
  }

  public ChassisSpeeds calculateChassisSpeeds(Translation2d targetGamepiece, Pose2d pose) {
    double rotationalErrorDegrees;
    double xVel;
    double yVel;
    double rotVel;

    double rotVelCorrection = 0;
    Rotation2d gamepieceDirection = new Rotation2d(targetGamepiece.getX(), targetGamepiece.getY());

    Rotation2d sourceAngle = Constants.zeroRotation2d;

    if (targetGamepiece == null) return null;

    rotationalErrorDegrees =
        Math.abs(gamepieceDirection.getDegrees() - pose.getRotation().getDegrees());
    Logger.recordOutput("rotationalErrorDegrees", rotationalErrorDegrees);

    if (isInSourceArea(targetGamepiece)) {
      sourceAngle =
          new Rotation2d(Constants.isRedAlliance() ? -2.103952069121958 : -1.0417663596685425);

      xVel = xKpSourceArea.get() * Math.cos(gamepieceDirection.getRadians());
      yVel = yKpSourceArea.get() * Math.sin(gamepieceDirection.getRadians());

      rotVel = rotController.calculate(pose.getRotation().getRadians(), sourceAngle.getRadians());
    } else {
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
        xVel =
            xController.calculate(pose.getX(), targetGamepiece.getX())
                / Math.max(rotationalErrorDegrees / allowedRotationalErrorDegreesValue, 1.0);
        yVel =
            yController.calculate(pose.getY(), targetGamepiece.getY())
                / Math.max(rotationalErrorDegrees / allowedRotationalErrorDegreesValue, 1.0);
      }

      rotVel =
          rotController.calculate(pose.getRotation().getRadians(), gamepieceDirection.getRadians());
    }

    Logger.recordOutput("DriveToGamepiece/rotationalErrorDegrees", rotationalErrorDegrees);
    Logger.recordOutput("DriveToGamepiece/xVel", xVel);
    Logger.recordOutput("DriveToGamepiece/yVel", yVel);
    Logger.recordOutput("DriveToGamepiece/rotVel", rotVel);

    Logger.recordOutput("ActiveCommands/DriveToGamepiece", true);

    return ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, pose.getRotation());
  }
}
