package frc.lib.team2930;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ShootingSolver {
  public static boolean DebugSpew = false;

  private final Translation3d Pspeaker;
  private final Translation3d PaxisOfRotationShooter;
  private final double shooterRPM;

  // private final double shootingTime;
  // private double startOfShootingTimestamp = Double.NaN;

  public record Solution(
      double timeToShoot, Rotation2d heading, double angularVel, Rotation2d pitch) {}

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
  public Pair<Rotation2d, Rotation2d> computeRobotYaw(
      double time,
      Translation2d robotPos,
      Translation2d robotVel,
      Translation2d robotAcceleration) {

    Logger.recordOutput("ShootingSolver/time", time);

    // var robotVelSpeaker = new Translation2d(0.0, 0.0).minus(robotVel);
    // var robotAccelerationSpeaker = new Translation2d(0.0, 0.0).minus(robotAcceleration);

    // double timeToShoot;
    // if (Double.isNaN(this.startOfShootingTimestamp)) {
    //   timeToShoot = shootingTime;
    // } else {
    //   timeToShoot = Math.max(0, shootingTime - (currentTime - startOfShootingTimestamp));
    // }

    var shooterSpeed =
        shooterRPM / 60.0 * Constants.ShooterConstants.Launcher.WHEEL_DIAMETER_METERS * Math.PI;

    var VnoteHorizontal = getNoteHorizVel(shooterSpeed, robotPos);

    var dPspeakerAxis = getAxisRelativePosition(robotPos);

    Logger.recordOutput("ShootingSolver/dPSpeakerAxis", dPspeakerAxis);

    Logger.recordOutput(
        "ShootingSolver/currentPos",
        new Pose2d(dPspeakerAxis.toTranslation2d(), new Rotation2d(0.0)));

    // Translation2d futureVel = robotVel.plus(robotAcceleration.times(time));

    //
    // The system of equations to solve is:
    //
    // Vn_x = (Vr_x + Vs * cos(targetTheta))
    // Vn_y = (Vr_y + Vs * sin(targetTheta))
    //
    // The Vn vector has to be aligned with dPspeaker vector, so
    //
    // Vn_x = c * dPspeakerAxisX
    // Vn_y = c * dPspeakerAxisY
    //
    // Substituting in the system of equations and dividing to get rid of 'c':
    //
    // dPspeakerAxisY   (Vr_y + Vs * sin(targetTheta))
    // -------------- = ------------------------------
    // dPspeakerAxisX   (Vr_x + Vs * cos(targetTheta))
    //
    // To verify that the condition is physically possible, we check that the note moves towards the
    // speaker:
    //
    // Vn_x = (Vr_x + Vs * cos(targetTheta)) > 0
    //
    var noteRelativeVel = new Translation2d(VnoteHorizontal, 0);

    Translation2d dPspeakreAxisFuture =
        dPspeakerAxis
            .toTranslation2d()
            .plus(robotVel.plus(robotAcceleration.times(0.5 * time)).times(time * -1.0));

    // Direction to the speaker from the note.
    double speakerHeading = Math.atan2(dPspeakreAxisFuture.getY(), dPspeakreAxisFuture.getX());

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

    // add PI to point back of robot to speaker
    double targetTheta = speakerHeading + thetaInNewFrame + Math.PI;

    if (DebugSpew) {
      double robotVelX = robotVel.getX();
      double robotVelY = robotVel.getY();

      System.out.printf("robotVelX:%s robotVelY:%s\n", robotVelX, robotVelY);
      System.out.printf(
          "VnoteHorizontal:%s targetTheta:%s\n", VnoteHorizontal, Math.toDegrees(targetTheta));

      System.out.printf(
          "speaker: Heading:%s  dx:%s dy:%s\n",
          Math.toDegrees(speakerHeading), dPspeakerAxis.getX(), dPspeakerAxis.getY());

      double noteX = robotVelX + VnoteHorizontal * Math.cos(targetTheta);
      double noteY = robotVelY + VnoteHorizontal * Math.sin(targetTheta);
      double nodeHeading = Math.atan2(noteY, noteX);
      System.out.printf(
          "nodeHeading: %s  noteX:%s noteY:%s\n", Math.toDegrees(nodeHeading), noteX, noteY);
    }

    // rotation velocity calculated from online derivative calculator:
    // https://www.derivative-calculator.net/#expr=asin%28-%28%28b%2Bcx%29%2Asin%28-atan%28%28d-fx-%281%2F2%29gx%5E2%29%2F%28a-bx-%281%2F2%29cx%5E2%29%29%29%2B%28f%2Bgx%29%2Acos%28-atan%28%28d-fx-%281%2F2%29gx%5E2%29%2F%28a-bx-%281%2F2%29cx%5E2%29%29%29%29%2Fh%29&simplify=1

    Translation2d futureRobotVel = robotVel.plus(robotAcceleration.times(time));
    double futureRobotVelX = futureRobotVel.getX();
    double futureRobotVelY = futureRobotVel.getY();

    Translation2d futureSpeakerVel = futureRobotVel.times(-1.0);
    double futureSpeakerVelX = futureSpeakerVel.getX();
    double futureSpeakerVelY = futureSpeakerVel.getY();

    Translation2d futureSpeakerOffset =
        dPspeakerAxis
            .toTranslation2d()
            .plus(robotVel.plus(robotAcceleration.times(0.5 * time)).times(-1.0 * time));
    double futureSpeakerOffsetX = futureSpeakerOffset.getX();
    double futureSpeakerOffsetY = futureSpeakerOffset.getY();

    Logger.recordOutput(
        "ShootingSolver/estimatedFuturePose",
        new Pose2d(
            robotPos.plus(robotVel.plus(robotAcceleration.times(0.5 * time)).times(time)),
            new Rotation2d(targetTheta)));

    double repeatedMath =
        Math.sqrt(
            (futureSpeakerOffsetY * futureSpeakerOffsetY)
                    / (futureSpeakerOffsetX * futureSpeakerOffsetX)
                + 1.0);

    var rotVel =
        -(-(futureRobotVelX * futureSpeakerVelY) / (futureSpeakerOffsetX * repeatedMath)
                - (robotAcceleration.getX() * futureSpeakerOffsetY)
                    / (futureSpeakerOffsetX * repeatedMath)
                + (futureSpeakerVelX * futureRobotVelX * futureSpeakerOffsetY)
                    / ((futureSpeakerOffsetX * futureSpeakerOffsetX) * repeatedMath)
                + robotAcceleration.getY() / repeatedMath
                + (futureRobotVelX
                        * futureSpeakerOffsetY
                        * ((2.0 * futureSpeakerVelY * futureSpeakerOffsetY)
                                / (futureSpeakerOffsetX * futureSpeakerOffsetX)
                            - (2.0
                                    * futureSpeakerVelX
                                    * (futureSpeakerOffsetY * futureSpeakerOffsetY))
                                / (futureSpeakerOffsetX
                                    * futureSpeakerOffsetX
                                    * futureSpeakerOffsetX)))
                    / (2.0
                        * futureSpeakerOffsetX
                        * Math.pow(
                            ((futureSpeakerOffsetY * futureSpeakerOffsetY)
                                    / (futureSpeakerOffsetX * futureSpeakerOffsetX)
                                + 1.0),
                            (3.0 / 2.0)))
                - (futureRobotVelY
                        * ((2.0 * futureSpeakerVelY * futureSpeakerOffsetY)
                                / (futureSpeakerOffsetX * futureSpeakerOffsetX)
                            - (2.0
                                    * futureSpeakerVelX
                                    * (futureSpeakerOffsetY * futureSpeakerOffsetY))
                                / (futureSpeakerOffsetX
                                    * futureSpeakerOffsetX
                                    * futureSpeakerOffsetX)))
                    / (2.0
                        * Math.pow(
                            ((futureSpeakerOffsetY * futureSpeakerOffsetY)
                                    / (futureSpeakerOffsetX * futureSpeakerOffsetX)
                                + 1.0),
                            (3.0 / 2.0))))
            / (VnoteHorizontal
                * Math.sqrt(
                    1
                        - Math.pow(
                                (futureRobotVelY / repeatedMath
                                    - (futureRobotVelX * futureSpeakerOffsetY)
                                        / (futureSpeakerOffsetX * repeatedMath)),
                                2)
                            / (VnoteHorizontal * VnoteHorizontal)));

    if (targetTheta >= Math.PI) {
      targetTheta -= Math.PI * 2.0;
    }

    return new Solution(new Rotation2d(targetTheta), new Rotation2d(rotVel));

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

    // Translation2d futureSpeakerOffset =
    //     dPspeakerAxis
    //         .toTranslation2d()
    //         .plus(robotVelSpeaker.plus(robotAccelerationSpeaker.times(time * 0.5)).times(time));

    // if (DebugSpew) {
    //   System.out.printf("ProbotFuture: %s\n", futurePos);
    // }

    // Logger.recordOutput(
    //     "ShootingSolver/futurePos", new Pose2d(futureSpeakerOffset, new Rotation2d(0.0)));

    // Archit's math for solving theta

    // double targetTheta =
    //     Math.acos(
    //             (((futureSpeakerOffset.getX() * futureVel.getY())
    //                         - (futureSpeakerOffset.getY() * futureVel.getX()))
    //                     * Math.hypot(futureSpeakerOffset.getX(), futureSpeakerOffset.getY()))
    //                 / (VnoteHorizontal
    //                     * (futureSpeakerOffset.getX() * futureSpeakerOffset.getX()
    //                         + futureSpeakerOffset.getY() * futureSpeakerOffset.getY())))
    //         + Math.atan(-futureSpeakerOffset.getX() / futureSpeakerOffset.getY());

    // Translation2d finalRobotVelSpeaker =
    // robotVelSpeaker.plus(robotAccelerationSpeaker.times(time));

    // double targetRotVel =
    //     -(((robotAcceleration.getY() * futureSpeakerOffset.getX())
    //                             / (VnoteHorizontal * futureSpeakerOffset.getY())
    //                         + (finalRobotVelSpeaker.getX() * futureVel.getY())
    //                             / (VnoteHorizontal * futureSpeakerOffset.getY())
    //                         - (finalRobotVelSpeaker.getY()
    //                                 * futureVel.getY()
    //                                 * futureSpeakerOffset.getX())
    //                             / (VnoteHorizontal
    //                                 * futureSpeakerOffset.getY()
    //                                 * futureSpeakerOffset.getY())
    //                         - robotAcceleration.getX() / VnoteHorizontal)
    //                     / Math.sqrt(
    //                         futureSpeakerOffset.getX()
    //                                 * futureSpeakerOffset.getX()
    //                                 / (futureSpeakerOffset.getY() * futureSpeakerOffset.getY())
    //                             + 1.0)
    //                 - (((2.0 * finalRobotVelSpeaker.getX() * futureSpeakerOffset.getX())
    //                                 / (futureSpeakerOffset.getY() * futureSpeakerOffset.getY())
    //                             - (2.0
    //                                     * finalRobotVelSpeaker.getY()
    //                                     * futureSpeakerOffset.getX()
    //                                     * futureSpeakerOffset.getX())
    //                                 / (futureSpeakerOffset.getY()
    //                                     * futureSpeakerOffset.getY()
    //                                     * futureSpeakerOffset.getY()))
    //                         * ((futureVel.getY() * futureSpeakerOffset.getX())
    //                                 / (VnoteHorizontal * futureSpeakerOffset.getY())
    //                             - futureVel.getX() / VnoteHorizontal))
    //                     / (2.0
    //                         * Math.pow(
    //                             (futureSpeakerOffset.getX()
    //                                     * futureSpeakerOffset.getX()
    //                                     / (futureSpeakerOffset.getY() *
    // futureSpeakerOffset.getY())
    //                                 + 1.0),
    //                             (3.0 / 2.0))))
    //             / Math.sqrt(
    //                 1.0
    //                     - Math.pow(
    //                             ((futureVel.getY() * futureSpeakerOffset.getX())
    //                                     / (VnoteHorizontal * futureSpeakerOffset.getY())
    //                                 - futureVel.getX() / VnoteHorizontal),
    //                             2.0)
    //                         / (futureSpeakerOffset.getX()
    //                                 * futureSpeakerOffset.getX()
    //                                 / (futureSpeakerOffset.getY() * futureSpeakerOffset.getY())
    //                             + 1.0))
    //         - (futureSpeakerOffset.getX() / futureSpeakerOffset.getY()
    //                 - (finalRobotVelSpeaker.getY() * futureSpeakerOffset.getX())
    //                     / (futureSpeakerOffset.getY() * futureSpeakerOffset.getY()))
    //             / (futureSpeakerOffset.getX()
    //                     * futureSpeakerOffset.getX()
    //                     / (futureSpeakerOffset.getY() * futureSpeakerOffset.getY())
    //                 + 1.0);

    // if (futureSpeakerOffset.getY() >= 0) {
    //   targetTheta += Math.PI;
    // }

    // if (targetTheta >= Math.PI) {
    //   targetTheta -= Math.PI * 2.0;
    // }

    // return new Solution(new Rotation2d(targetTheta), new Rotation2d(-targetRotVel));
  }

  private double getNoteHorizVel(double vel, Translation2d robotPos) {

    var dPspeakerAxis = getAxisRelativePosition(robotPos);

    var xyDistToSpeaker = Math.hypot(dPspeakerAxis.getX(), dPspeakerAxis.getY());
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

  public static void logTime(double time) {
    Logger.recordOutput("ShootingSolver/loggedTime", time);
  }
}
