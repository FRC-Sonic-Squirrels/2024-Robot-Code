package frc.lib.team2930;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;

public class ShootingSolver {
  private final Translation3d Pspeaker;
  private final Translation3d PaxisOfRotationShooter;
  private final double shooterSpeed;
  private final double shootingTime;
  private double startOfShootingTimestamp = Double.NaN;

  public ShootingSolver(
      Translation3d Pspeaker,
      Translation3d PaxisOfRotationShooter,
      double shooterSpeed,
      double shootingTime) {
    this.Pspeaker = Pspeaker;
    this.PaxisOfRotationShooter = PaxisOfRotationShooter;
    this.shooterSpeed = shooterSpeed;
    this.shootingTime = shootingTime;
  }

  public void startShooting(double timestamp) {
    startOfShootingTimestamp = timestamp;
  }

  public void endShooting() {
    startOfShootingTimestamp = Double.NaN;
  }

  /**
   * @return Rotation2d 1: robot theta Rotation2d 2: shooter pitch
   */
  public Pair<Rotation2d, Rotation2d> computeAngles(
      double currentTime, Pose2d robotPose, Translation2d robotVel) {
    double timeToShoot;
    if (Double.isNaN(this.startOfShootingTimestamp)) {
      timeToShoot = shootingTime;
    } else {
      timeToShoot = Math.max(0, shootingTime - (currentTime - startOfShootingTimestamp));
    }

    // 3d position of robot
    var Probot = GeometryUtil.translation2dTo3d(robotPose.getTranslation());

    // 3d velocity vector of robot
    var Vrobot = GeometryUtil.translation2dTo3d(robotVel);

    // position of robot when note leaves shooter
    var ProbotFuture = Probot.plus(Vrobot.times(timeToShoot));

    // vector pointing from axis to speaker at time that note leaves shooter
    var dPspeaker = Pspeaker.minus(ProbotFuture);

    // velocity theta of gamepiece
    var thetaNote = Math.atan2(dPspeaker.getY(), dPspeaker.getX());

    // current heading of robot
    Rotation3d heading = new Rotation3d(0.0, 0.0, thetaNote);

    // robot relative translation of axis of rotation of shooter
    var Paxis = PaxisOfRotationShooter.rotateBy(heading);

    // position of axis when note leaves shooter
    var PaxisFuture = ProbotFuture.plus(Paxis);

    var dPspeakerAxis = Pspeaker.minus(PaxisFuture);

    // horizontal dist to speaker
    double dPspeakerAxisX = dPspeakerAxis.getX();
    double dPspeakerAxisY = dPspeakerAxis.getY();

    var xyDistToSpeaker = Math.hypot(dPspeakerAxisX, dPspeakerAxisY);

    // desired shooter pivot pitch
    var pitchNote = Math.atan2(dPspeakerAxis.getZ(), xyDistToSpeaker);

    var VnoteHorizontal = shooterSpeed * Math.cos(pitchNote);

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
    double targetTheta;

    double robotVelX = robotVel.getX();
    double robotVelY = robotVel.getY();

    if (Math.abs(dPspeakerAxisY) < 0.001) {
      //
      // Speaker right in front of robot
      //
      // -> Vn_y = 0
      // ->  (Vr_y + Vs * sin(targetTheta))
      // -> -Vr_y / Vs = sin(targetTheta))
      // -> targetTheta = arcsin(-Vr_y / Vs)
      //
      targetTheta = Math.asin(-robotVelY / VnoteHorizontal);
    } else {
      var a = -dPspeakerAxisX / dPspeakerAxisY;

      var b =
          (dPspeakerAxisX * robotVelY - dPspeakerAxisY * robotVelX)
              / (VnoteHorizontal * dPspeakerAxisY);

      // Archit's math for solving theta
      targetTheta = Math.acos(b / Math.sqrt(1.0 + a * a)) + Math.atan(a);

      if (dPspeakerAxisY >= 0) {
        targetTheta += Math.PI;
      }
    }

    return new Pair<>(new Rotation2d(targetTheta), new Rotation2d(pitchNote));
  }
}
