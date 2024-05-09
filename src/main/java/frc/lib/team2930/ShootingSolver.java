package frc.lib.team2930;

import edu.wpi.first.math.geometry.*;

public class ShootingSolver {
  public static final int DebugSpew = 0;

  private static final LoggerGroup logGroup = LoggerGroup.build("ShootingSolver");
  private static final LoggerEntry.Decimal logThetaNote = logGroup.buildDecimal("thetaNote");
  private static final LoggerEntry.Decimal logSpeakerHeading =
      logGroup.buildDecimal("speakerHeading");

  private final Translation3d Pspeaker;
  private final Translation3d PaxisOfRotationShooter;
  private final Translation3d PfrontOfShooter;
  private final double shooterSpeed;
  private final double shootingTime;
  private final boolean correctForLateralMotion;
  private double startOfShootingTimestamp = Double.NaN;
  private boolean doneShooting;

  public record Solution(
      double timeToShoot,
      Rotation2d heading,
      double rotationSpeed,
      Rotation2d pitch,
      double xyDistance,
      Translation3d xyOffset) {}

  public ShootingSolver(
      Translation3d Pspeaker,
      Translation3d PaxisOfRotationShooter,
      Translation3d PfrontOfShooter,
      double shooterSpeed,
      double shootingTime,
      boolean correctForLateralMotion) {
    this.Pspeaker = Pspeaker;
    this.PaxisOfRotationShooter = PaxisOfRotationShooter;
    this.PfrontOfShooter = PfrontOfShooter;
    this.shooterSpeed = shooterSpeed;
    this.shootingTime = shootingTime;
    this.correctForLateralMotion = correctForLateralMotion;
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
   * Accepts current robot geometry and returns shooting parameters
   *
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
    if (DebugSpew > 0) {
      System.out.printf("ProbotFuture: %s\n", ProbotFuture);
    }

    // Vector pointing from robot to speaker at time that note leaves shooter
    var dPspeaker = Pspeaker.minus(ProbotFuture);

    // Direction of the note to face the speaker.
    var thetaNote = Math.atan2(dPspeaker.getY(), dPspeaker.getX());
    logThetaNote.info(Math.toDegrees(thetaNote));

    // Robot relative translation of axis of rotation of shooter
    var Paxis = PaxisOfRotationShooter.rotateBy(new Rotation3d(0.0, 0.0, thetaNote));

    // Position of axis when note leaves shooter
    var PaxisFuture = ProbotFuture.plus(Paxis);

    // Vector from the note to the speaker. We need to align with this vector to score.
    var dPspeakerAxis = Pspeaker.minus(PaxisFuture);

    double dPspeakerAxisX = dPspeakerAxis.getX();
    double dPspeakerAxisY = dPspeakerAxis.getY();

    // Direction to the speaker from the note.
    double speakerHeading = Math.atan2(dPspeakerAxisY, dPspeakerAxisX);

    logSpeakerHeading.info(Math.toDegrees(speakerHeading));

    // Horizontal dist to speaker
    var xyDistToSpeaker = Math.hypot(dPspeakerAxisX, dPspeakerAxisY);
    if (DebugSpew > 0) {
      System.out.println();
      System.out.printf("Speaker: heading:%.1f\n", Math.toDegrees(speakerHeading));
      System.out.printf("         dx:%.1f\n", dPspeakerAxisX);
      System.out.printf("         dy:%.1f\n", dPspeakerAxisY);
      System.out.printf("         xyDistToSpeaker: %.1f\n", xyDistToSpeaker);

      System.out.printf("Robot: Vx:%.1f\n", robotVel.getX());
      System.out.printf("       Vy:%.1f\n", robotVel.getY());
    }

    // Desired pitch of the note trajectory
    double pitchNote = Math.atan2(dPspeakerAxis.getZ(), xyDistToSpeaker);

    // We start assuming the pitch of the shooter is going to be the pitch of the trajectory
    double shooterPitch = pitchNote;

    // Maximum number of approximations to perform
    int maxApproximations = 10;

    double targetTheta = speakerHeading;

    if (correctForLateralMotion) {
      // We need to compute this, which depends on shooterPitch, so we need to use successive
      // approximations
      double yawInNewFrame;

      while (true) {
        //
        // The system of equations to solve for cancelling the horizontal component of the robot
        // velocity is:
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
        // To verify that the condition is physically possible, we check that the note moves towards
        // the
        // speaker:
        //
        // Vn_x = (Vr_x + Vs * cos(targetTheta)) > 0
        //
        if (DebugSpew > 1) {
          System.out.printf(
              "shooterPitch:%.1f  maxApproximations:%d\n",
              Math.toDegrees(shooterPitch), maxApproximations);
        }
        var VnoteVertical = shooterSpeed * Math.sin(shooterPitch);
        var VnoteHorizontal = shooterSpeed * Math.cos(shooterPitch);
        var noteRelativeVel = new Translation2d(VnoteHorizontal, 0);

        Rotation2d frameRotation = Rotation2d.fromRadians(-speakerHeading);
        var robotVelInNewFrame = robotVel.rotateBy(frameRotation);
        var noteRelativeVelInNewFrame = noteRelativeVel.rotateBy(frameRotation);

        double robotVelXInNewFrame = robotVelInNewFrame.getX();
        double robotVelYInNewFrame = robotVelInNewFrame.getY();
        double noteVelXInNewFrame = noteRelativeVelInNewFrame.getX();
        double noteVelYInNewFrame = noteRelativeVelInNewFrame.getY();

        if (DebugSpew > 1) {
          System.out.printf("NewFrame: robotVelX:%.1f\n", robotVelXInNewFrame);
          System.out.printf("          robotVelY:%.1f\n", robotVelYInNewFrame);
          System.out.printf("          noteVelX :%.1f\n", noteVelXInNewFrame);
          System.out.printf("          noteVelY :%.1f\n", noteVelYInNewFrame);
        }

        yawInNewFrame = Math.asin(-robotVelYInNewFrame / VnoteHorizontal);

        if (Double.isNaN(yawInNewFrame)
            || (robotVelXInNewFrame + VnoteHorizontal * Math.cos(yawInNewFrame) <= 0)) {
          return null;
        }

        var VnoteHorizontal_FieldRelative =
            robotVelXInNewFrame + VnoteHorizontal * Math.cos(yawInNewFrame);

        if (DebugSpew > 2) // Check equation outputs.
        {
          double robotVelX = robotVel.getX();
          double robotVelY = robotVel.getY();
          double targetTheta2 = speakerHeading + yawInNewFrame;

          double noteX = robotVelX + VnoteHorizontal * Math.cos(targetTheta2);
          double noteY = robotVelY + VnoteHorizontal * Math.sin(targetTheta2);
          double nodeHeading = Math.atan2(noteY, noteX);
          System.out.printf(
              "nodeHeading: %.1f  noteX:%.1f noteY:%.1f\n",
              Math.toDegrees(nodeHeading), noteX, noteY);
        }

        // We have to verify that after the adjustment the note is still pointing to the speaker

        // Desired pitch of the note trajectory
        double pitchNoteNew = Math.atan2(VnoteVertical, VnoteHorizontal_FieldRelative);

        if (DebugSpew > 1) {
          System.out.printf("Note: NewPitch:%.1f\n", Math.toDegrees(pitchNoteNew));
          System.out.printf("      DesiredPitch:%.1f\n", Math.toDegrees(pitchNote));
          System.out.printf("      Yaw:%.1f\n", Math.toDegrees(yawInNewFrame));
          System.out.printf("      VnoteVertical:%.1f\n", VnoteVertical);
          System.out.printf("      VnoteHorizontal:%.1f\n", VnoteHorizontal);
          System.out.printf(
              "      VnoteHorizontalFieldRelative:%.1f\n", VnoteHorizontal_FieldRelative);
        }

        if (Math.toDegrees(Math.abs(pitchNoteNew - pitchNote)) < 0.1) {
          break;
        }

        shooterPitch -= (pitchNoteNew - pitchNote);

        if (maxApproximations-- <= 0) {
          break;
        }
      }

      targetTheta += yawInNewFrame;
    }

    var PaxisFinal = PaxisOfRotationShooter.rotateBy(new Rotation3d(0.0, 0.0, targetTheta));
    var PfrontFinal = PfrontOfShooter.rotateBy(new Rotation3d(0.0, 0.0, targetTheta));

    double axisDistance = Pspeaker.getDistance(ProbotFuture.plus(PaxisFinal));
    double frontDistance = Pspeaker.getDistance(ProbotFuture.plus(PfrontFinal));

    if (frontDistance > axisDistance) {
      targetTheta += Math.PI;
    }

    var currentHeading = robotPose.getRotation().getRadians();

    var deltaAngle = GeometryUtil.optimizeRotation(targetTheta - currentHeading);

    var rateOfRotation = deltaAngle / Math.max(0.01, timeToShoot);

    return new Solution(
        timeToShoot,
        new Rotation2d(targetTheta),
        rateOfRotation,
        new Rotation2d(shooterPitch),
        xyDistToSpeaker,
        dPspeaker);
  }
}
