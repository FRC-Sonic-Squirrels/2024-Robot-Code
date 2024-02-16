package frc.lib.team2930;

import edu.wpi.first.math.geometry.*;

public class ShootingSolver {
  public static boolean DebugSpew;

  private final Translation3d Pspeaker;
  private final Translation3d PaxisOfRotationShooter;
  private final Translation3d PfrontOfShooter;
  private final double shooterSpeed;
  private final double shootingTime;
  private double startOfShootingTimestamp = Double.NaN;
  private boolean doneShooting;

  public record Solution(
      double timeToShoot, Rotation2d heading, double rotationSpeed, Rotation2d pitch) {}

  public ShootingSolver(
      Translation3d Pspeaker,
      Translation3d PaxisOfRotationShooter,
      Translation3d PfrontOfShooter,
      double shooterSpeed,
      double shootingTime) {
    this.Pspeaker = Pspeaker;
    this.PaxisOfRotationShooter = PaxisOfRotationShooter;
    this.PfrontOfShooter = PfrontOfShooter;
    this.shooterSpeed = shooterSpeed;
    this.shootingTime = shootingTime;
  }

  public void startShooting(double timestamp) {
    startOfShootingTimestamp = timestamp;
  }

  public void endShooting() {
    startOfShootingTimestamp = Double.NaN;
    doneShooting = false;
  }

  public boolean isShooting() {
    return Double.isFinite(startOfShootingTimestamp) && !doneShooting;
  }

  /**
   * @return robot theta and shooter pitch
   */
  public Solution computeAngles(double currentTime, Pose2d robotPose, Translation2d robotVel) {
    double timeToShoot;
    if (Double.isNaN(this.startOfShootingTimestamp)) {
      timeToShoot = shootingTime;
    } else {
      timeToShoot = shootingTime - (currentTime - startOfShootingTimestamp);
      if (timeToShoot < 0) {
        doneShooting = true;
        timeToShoot = 0;
      }
    }

    // 3d position of robot
    var Probot = GeometryUtil.translation2dTo3d(robotPose.getTranslation());

    // 3d velocity vector of robot
    var Vrobot = GeometryUtil.translation2dTo3d(robotVel);

    // Position of robot when note leaves shooter
    var ProbotFuture = Probot.plus(Vrobot.times(timeToShoot));
    if (DebugSpew) {
      System.out.printf("ProbotFuture: %s\n", ProbotFuture);
    }

    // Vector pointing from robot to speaker at time that note leaves shooter
    var dPspeaker = Pspeaker.minus(ProbotFuture);

    // Direction of the note to face the speaker.
    var thetaNote = Math.atan2(dPspeaker.getY(), dPspeaker.getX());

    // Robot relative translation of axis of rotation of shooter
    var Paxis = PaxisOfRotationShooter.rotateBy(new Rotation3d(0.0, 0.0, thetaNote));

    // Position of axis when note leaves shooter
    var PaxisFuture = ProbotFuture.plus(Paxis);

    // Vector from the note to the speaker. We need to align with this vector to score.
    var dPspeakerAxis = Pspeaker.minus(PaxisFuture);

    double dPspeakerAxisX = dPspeakerAxis.getX();
    double dPspeakerAxisY = dPspeakerAxis.getY();

    // Horizontal dist to speaker
    var xyDistToSpeaker = Math.hypot(dPspeakerAxisX, dPspeakerAxisY);
    if (DebugSpew) {
      System.out.printf("xyDistToSpeaker: %s\n", xyDistToSpeaker);
    }

    // Desired shooter pivot pitch
    // FIXME: BAD CALCULATION FOR SHOOTER PITCH. NEW APPROACH NECESSARY
    var pitchNote = Math.atan2(dPspeakerAxis.getZ(), xyDistToSpeaker);

    // Code using Archit's formula for pitch of shooter.
    // var a = -xyDistToSpeaker / dPspeakerAxis.getZ();
    // var b =
    //     -robotVel
    //             .rotateBy(
    //                 Rotation2d.fromRadians(Math.atan2(dPspeakerAxisY,
    // dPspeakerAxisX)).unaryMinus())
    //             .getX()
    //         / shooterSpeed;
    // var insideCos = b * Math.sqrt(1.0 + a * a) / (a * a + 1.0);
    // double pitchNote;
    // if (Math.abs(insideCos) <= 1.0) {
    //   pitchNote = Math.acos(insideCos) + Math.atan(a);
    // } else {
    //   pitchNote = Math.atan2(dPspeakerAxis.getZ(), xyDistToSpeaker);
    // }

    // Direction to the speaker from the note.
    double speakerHeading = Math.atan2(dPspeakerAxisY, dPspeakerAxisX);

    var VnoteHorizontal = shooterSpeed * Math.cos(pitchNote);

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
    //     Vn_y = 0
    // ->  Vr_y + Vs * sin(targetTheta) = 0
    // -> -Vr_y / Vs = sin(targetTheta))
    // ->  targetTheta = arcsin(-Vr_y / Vs)
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

    var PaxisFinal = PaxisOfRotationShooter.rotateBy(new Rotation3d(0.0, 0.0, targetTheta));
    var PfrontFinal = PfrontOfShooter.rotateBy(new Rotation3d(0.0, 0.0, targetTheta));

    double axisDistance = Pspeaker.getDistance(ProbotFuture.plus(PaxisFinal));
    double frontDistance = Pspeaker.getDistance(ProbotFuture.plus(PfrontFinal));

    if (frontDistance > axisDistance) {
      targetTheta += Math.PI;
    }

    var currentHeading = robotPose.getRotation().getRadians();

    var deltaAngle = targetTheta - currentHeading;
    while (deltaAngle <= -Math.PI) deltaAngle += Math.PI * 2;
    while (deltaAngle >= Math.PI) deltaAngle -= Math.PI * 2;

    var rateOfRotation = deltaAngle / Math.max(0.01, timeToShoot);

    return new Solution(
        timeToShoot, new Rotation2d(targetTheta), rateOfRotation, new Rotation2d(pitchNote));
  }
}
