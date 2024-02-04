package frc.lib.team2930;

import edu.wpi.first.math.geometry.*;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShootingSolver {
  public static boolean DebugSpew;

  private final Translation3d Pspeaker;
  private final Translation3d PaxisOfRotationShooter;
  private final double shooterRPM;

  // private final double shootingTime;
  // private double startOfShootingTimestamp = Double.NaN;

  public record Solution(Rotation2d heading, Rotation2d pitch) {}

  public ShootingSolver(
      Translation3d Pspeaker, Translation3d PaxisOfRotationShooter, double shooterRPM) {
    this.Pspeaker = Pspeaker;
    this.PaxisOfRotationShooter = PaxisOfRotationShooter;
    this.shooterRPM = shooterRPM;
    // this.shootingTime = shootingTime;
  }

  // public void startShooting(double timestamp) {
  //   startOfShootingTimestamp = timestamp;
  // }

  // public void endShooting() {
  //   startOfShootingTimestamp = Double.NaN;
  // }

  /**
   * @return First: target robot theta Second: target robot rotational velocity
   */
  public Solution computeRobotYaw(
      DoubleSupplier time,
      Translation2d robotPos,
      Translation2d robotVel,
      Translation2d robotAcceleration) {

    var robotVelSpeaker = new Translation2d(0.0, 0.0).minus(robotVel);
    var robotAccelerationSpeaker = new Translation2d(0.0, 0.0).minus(robotAcceleration);
    // double timeToShoot;
    // if (Double.isNaN(this.startOfShootingTimestamp)) {
    //   timeToShoot = shootingTime;
    // } else {
    //   timeToShoot = Math.max(0, shootingTime - (currentTime - startOfShootingTimestamp));
    // }

    var shooterSpeed =
        shooterRPM / 60.0 * Constants.ShooterConstants.Launcher.WHEEL_DIAMETER_METERS * Math.PI;

    var VnoteHorizontal = getNoteHorizVel(shooterSpeed, robotPos);

    var dPSpeakerAxis = getAxisRelativePosition(robotPos);

    Logger.recordOutput("ShootingSolver/dPSpeakerAxis", dPSpeakerAxis);
    //
    // The system of equations to solve is:
    //
    // Vn_x = (Vr_x + Vs * cos(targetTheta))
    // Vn_y = (Vr_y + Vs * sin(targetTheta))
    //
    // The Vn vector has to be aligned with dPspeakerAxis vector, so
    //
    // Vn_x = c * dPspeakerAxisX
    // Vn_y = c * dPspeakerAxisY
    //
    // To make it easier, we rotate the reference frame such that the Y velocity will be zero:
    //
    // Vn_y = 0
    // ->  Vr_y + Vs * sin(targetTheta) = 0
    // -> -Vr_y / Vs = sin(targetTheta))
    // -> targetTheta = arcsin(-Vr_y / Vs)
    //
    // To verify that the condition is physically possible, we check that the note moves towards the
    // speaker:
    //
    // Vn_x = (Vr_x + Vs * cos(targetTheta)) > 0
    //
    var noteRelativeVel = new Translation2d(VnoteHorizontal, 0);

    Rotation2d frameRotation = Rotation2d.fromRadians(-speakerHeading);
    var robotVelInNewFrame = robotVel.rotateBy(frameRotation);
    var noteRelativeVelInNewFrame = noteRelativeVel.rotateBy(frameRotation);

    double robotVelXInNewFrame = robotVelInNewFrame.getX();
    double robotVelYInNewFrame = robotVelInNewFrame.getY();
    double noteVelYInNewFrame = noteRelativeVelInNewFrame.getY();

    if (DebugSpew) {
      System.out.printf(
          "robotVelYInNewFrame: %s  noteVelYInNewFrame:%s\n",
          robotVelYInNewFrame, noteVelYInNewFrame);
    }

    double thetaInNewFrame = Math.asin(-robotVelYInNewFrame / VnoteHorizontal);

    if (Double.isNaN(thetaInNewFrame)
        || (robotVelXInNewFrame + VnoteHorizontal * Math.cos(thetaInNewFrame) <= 0)) {
      return null;
    }

    double targetTheta = speakerHeading + thetaInNewFrame;

    if (DebugSpew) {
      double robotVelX = robotVel.getX();
      double robotVelY = robotVel.getY();

      System.out.printf("robotVelX:%s robotVelY:%s\n", robotVelX, robotVelY);
      System.out.printf(
          "VnoteHorizontal:%s targetTheta:%s\n", VnoteHorizontal, Math.toDegrees(targetTheta));

      System.out.printf(
          "speaker: Heading:%s  dx:%s dy:%s\n",
          Math.toDegrees(speakerHeading), dPspeakerAxisX, dPspeakerAxisY);

      double noteX = robotVelX + VnoteHorizontal * Math.cos(targetTheta);
      double noteY = robotVelY + VnoteHorizontal * Math.sin(targetTheta);
      double nodeHeading = Math.atan2(noteY, noteX);
      System.out.printf(
          "nodeHeading: %s  noteX:%s noteY:%s\n", Math.toDegrees(nodeHeading), noteX, noteY);
    }

    return new Solution(new Rotation2d(targetTheta), new Rotation2d(0.0));

    // if (Math.abs(dPspeakerAxisY) < 0.001) {
    //   //
    //   // Speaker right in front of robot
    //   //
    //   // -> Vn_y = 0
    //   // ->  (Vr_y + Vs * sin(targetTheta))
    //   // -> -Vr_y / Vs = sin(targetTheta))
    //   // -> targetTheta = arcsin(-Vr_y / Vs)
    //   //
    //   targetTheta = Math.asin(-robotVelY / VnoteHorizontal);
    // } else {
    //   var a = -dPspeakerAxisX / dPspeakerAxisY;

    //   var b =
    //       (dPspeakerAxisX * robotVelY - dPspeakerAxisY * robotVelX)
    //           / (VnoteHorizontal * dPspeakerAxisY);

    //   // Archit's math for solving theta
    //   targetTheta = Math.acos(b / Math.sqrt(1.0 + a * a)) + Math.atan(a);
    // }

    Translation2d futurePos =
        dPSpeakerAxis
            .toTranslation2d()
            .plus(
                robotVelSpeaker
                    .plus(robotAccelerationSpeaker.times(time.getAsDouble() * 0.5))
                    .times(time.getAsDouble()));

    if (DebugSpew) {
      System.out.printf("ProbotFuture: %s\n", futurePos);
    }

    Logger.recordOutput("ShootingSolver/time", time.getAsDouble());

    Logger.recordOutput("ShootingSolver/futurePos", new Pose2d(futurePos, new Rotation2d(0.0)));
    Logger.recordOutput(
        "ShootingSolver/currentPos",
        new Pose2d(dPSpeakerAxis.toTranslation2d(), new Rotation2d(0.0)));

    Translation2d futureVel = robotVel.plus(robotAcceleration.times(time.getAsDouble()));

    // Archit's math for solving theta
    // double targetTheta =
    //     Math.acos(
    //             (futureVel.getY() * (futurePos.getX()) - futureVel.getX() * futurePos.getY())
    //                 / (VnoteHorizontal
    //                     * (futurePos.getY())
    //                     * Math.sqrt(
    //                         futurePos.getX()
    //                                 * futurePos.getX()
    //                                 / (futurePos.getY() * futurePos.getY())
    //                             + 1.0)))
    //         - Math.atan(futurePos.getX() / futurePos.getY());

    double targetTheta =
        Math.acos(
                (((futurePos.getX() * futureVel.getY()) - (futurePos.getY() * futureVel.getX()))
                        * Math.hypot(futurePos.getX(), futurePos.getY()))
                    / (VnoteHorizontal
                        * (futurePos.getX() * futurePos.getX()
                            + futurePos.getY() * futurePos.getY())))
            + Math.atan(-futurePos.getX() / futurePos.getY());

    Translation2d finalRobotVelSpeaker =
        robotVelSpeaker.plus(robotAccelerationSpeaker.times(time.getAsDouble()));

    double targetRotVel =
        -(((robotAcceleration.getY() * futurePos.getX()) / (VnoteHorizontal * futurePos.getY())
                            + (finalRobotVelSpeaker.getX() * futureVel.getY())
                                / (VnoteHorizontal * futurePos.getY())
                            - (finalRobotVelSpeaker.getY() * futureVel.getY() * futurePos.getX())
                                / (VnoteHorizontal * futurePos.getY() * futurePos.getY())
                            - robotAcceleration.getX() / VnoteHorizontal)
                        / Math.sqrt(
                            futurePos.getX()
                                    * futurePos.getX()
                                    / (futurePos.getY() * futurePos.getY())
                                + 1.0)
                    - (((2.0 * finalRobotVelSpeaker.getX() * futurePos.getX())
                                    / (futurePos.getY() * futurePos.getY())
                                - (2.0
                                        * finalRobotVelSpeaker.getY()
                                        * futurePos.getX()
                                        * futurePos.getX())
                                    / (futurePos.getY() * futurePos.getY() * futurePos.getY()))
                            * ((futureVel.getY() * futurePos.getX())
                                    / (VnoteHorizontal * futurePos.getY())
                                - futureVel.getX() / VnoteHorizontal))
                        / (2.0
                            * Math.pow(
                                (futurePos.getX()
                                        * futurePos.getX()
                                        / (futurePos.getY() * futurePos.getY())
                                    + 1.0),
                                (3.0 / 2.0))))
                / Math.sqrt(
                    1.0
                        - Math.pow(
                                ((futureVel.getY() * futurePos.getX())
                                        / (VnoteHorizontal * futurePos.getY())
                                    - futureVel.getX() / VnoteHorizontal),
                                2.0)
                            / (futurePos.getX()
                                    * futurePos.getX()
                                    / (futurePos.getY() * futurePos.getY())
                                + 1.0))
            - (futurePos.getX() / futurePos.getY()
                    - (finalRobotVelSpeaker.getY() * futurePos.getX())
                        / (futurePos.getY() * futurePos.getY()))
                / (futurePos.getX() * futurePos.getX() / (futurePos.getY() * futurePos.getY())
                    + 1.0);

    Logger.recordOutput(
        "ShootingSolver/w",
        Math.acos(
            (((futurePos.getX() * futureVel.getY()) - (futurePos.getY() * futureVel.getX()))
                    * Math.hypot(futurePos.getX(), futurePos.getY()))
                / (VnoteHorizontal
                    * (futurePos.getX() * futurePos.getX()
                        + futurePos.getY() * futurePos.getY()))));

    Logger.recordOutput("ShootingSolver/v", Math.atan(-futurePos.getX() / futurePos.getY()));
    Logger.recordOutput("ShootingSolver/vInside", -futurePos.getX() / futurePos.getY());

    if (futurePos.getY() >= 0) {
      targetTheta += Math.PI;
    }

    if (targetTheta >= Math.PI) {
      targetTheta -= Math.PI * 2.0;
    }

    return new Solution(new Rotation2d(targetTheta), new Rotation2d(-targetRotVel));
  }

  private double getNoteHorizVel(double vel, Translation2d robotPos) {

    var dPspeakerAxis = getAxisRelativePosition(robotPos);

    var xyDistToSpeaker = Math.hypot(dPspeakerAxis.getX(), dPspeakerAxis.getY());
    if (DebugSpew) {
      System.out.printf("xyDistToSpeaker: %s\n", xyDistToSpeaker);
    }
    // shooter pivot pitch
    var pitchNote = Math.atan2(dPspeakerAxis.getZ(), xyDistToSpeaker);
    return vel * Math.cos(pitchNote);
  }

  private Translation3d getAxisRelativePosition(Translation2d robotPos) {
    // 3d position of robot
    var Probot = GeometryUtil.translation2dTo3d(robotPos);

    // vector pointing from axis to speaker at time that note leaves shooter
    var dPspeaker = Pspeaker.minus(Probot);

    // velocity theta of gamepiece
    var thetaNote = Math.atan2(dPspeaker.getY(), dPspeaker.getX());

    // current heading of robot
    Rotation3d heading = new Rotation3d(0.0, 0.0, thetaNote);

    // robot relative translation of axis of rotation of shooter
    var PaxisRobotRel = PaxisOfRotationShooter.rotateBy(heading);

    // position of axis when note leaves shooter
    var Paxis = Probot.plus(PaxisRobotRel);

    return Pspeaker.minus(Paxis);
  }

  public void logTime(double time) {
    Logger.recordOutput("ShootingSolver/loggedTime", time);
  }
}
