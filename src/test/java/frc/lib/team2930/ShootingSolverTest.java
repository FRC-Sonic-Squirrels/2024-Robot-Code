package frc.lib.team2930;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.*;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

public class ShootingSolverTest {
  public static boolean debugCode;

  @Test()
  public void testOrbit() {
    Translation3d Pspeaker = new Translation3d(0, 0, 10);
    Translation3d PaxisOfRotationShooter = new Translation3d(0, 0, 0);
    Translation3d PfrontOfShooter = new Translation3d(0, -1, 0);
    double shootingTime = 1;

    var robotVel = new Translation2d(10, 0);
    var robotPos = new Translation2d(0, 10);

    //
    // Create a trajectory tangential to the speaker, rotating it around the speaker,
    // testing different shooter speeds.
    //
    for (double shooterSpeed = 10; shooterSpeed < 60; shooterSpeed += 10) {
      var solver =
          new ShootingSolver(
              Pspeaker, PaxisOfRotationShooter, PfrontOfShooter, shooterSpeed, shootingTime, true);

      for (double angle = 0; angle < 360; angle += 45) {
        var rot = Rotation2d.fromDegrees(angle);

        var robotPosRotated = robotPos.rotateBy(rot);
        var robotVelRotated = robotVel.rotateBy(rot);

        var robotPosRotatedAndTranslated = robotPosRotated.minus(robotVelRotated);
        var robotPose = new Pose2d(robotPosRotatedAndTranslated, Constants.zeroRotation2d);

        var res = solver.computeAngles(0, robotPose, robotVelRotated);

        double VnoteHorizontal = shooterSpeed * Math.cos(Math.toRadians(45));
        if (VnoteHorizontal < 10) {
          assertNull(res);
        } else {

          assertNotNull(res);

          if (debugCode) {
            System.out.printf("Result robotPose: %s\n", robotPose);
            System.out.printf("Result robotVelRotated: %s\n", robotVelRotated);
            System.out.printf("Result Pitch: %.1f\n", res.pitch().getDegrees());
            System.out.printf("Result Heading: %.1f\n", res.heading().getDegrees());
          }

          var noteStart3d = new Translation3d(shooterSpeed, 0, 0);

          var noteMid3d = noteStart3d.rotateBy(new Rotation3d(0, -res.pitch().getRadians(), 0));

          var noteVel3d =
              noteMid3d.rotateBy(new Rotation3d(0, 0, res.heading().getRadians() + Math.PI));

          var robotVel3d = GeometryUtil.translation2dTo3d(robotVelRotated);

          var noteVelTotal = robotVel3d.plus(noteVel3d);

          if (debugCode) {
            System.out.printf("noteStart3d: %s\n", noteStart3d);
            System.out.printf("noteMid3d: %s\n", noteMid3d);
            System.out.printf("noteVel3d: %s\n", noteVel3d);
            System.out.printf("robotVel3d: %s\n", robotVel3d);
            System.out.printf("noteVelTotal: %s\n", noteVelTotal);
            System.out.printf("totalSpeed: %.1f\n", noteVelTotal.getNorm());
            System.out.printf(
                "heading: %.1f\n",
                Math.toDegrees(Math.atan2(noteVelTotal.getY(), noteVelTotal.getX())));
          }

          var noteHorizontalReal = Math.hypot(noteVelTotal.getX(), noteVelTotal.getY());
          var noteVerticalReal = noteVelTotal.getZ();
          assertEquals(45, Math.toDegrees(Math.atan2(noteVerticalReal, noteHorizontalReal)), 0.1);
        }
      }
    }
  }

  public void printSpeedSweep() {
    Translation3d Pspeaker = new Translation3d(0, 0, 10);
    Translation3d PaxisOfRotationShooter = new Translation3d(0, 0, 0);
    Translation3d PfrontOfShooter = new Translation3d(0, 0, 0);
    double shootingTime = 1;

    for (double shooterSpeed = 10; shooterSpeed <= 50; shooterSpeed += 10) {
      var solver =
          new ShootingSolver(
              Pspeaker, PaxisOfRotationShooter, PfrontOfShooter, shooterSpeed, shootingTime, true);
      for (double i = -10; i <= 10; i += 0.1) {
        var res =
            solver.computeAngles(0, new Pose2d(-i, 10, new Rotation2d(0)), new Translation2d(i, 0));

        System.out.printf(
            "%3.1f: ROR:%3.1f heading:%3.1f pitch:%3.1f shooter:%2.1f\n",
            i,
            res != null ? res.rotationSpeed() : Double.NaN,
            res != null ? res.heading().getDegrees() : Double.NaN,
            res != null ? res.pitch().getDegrees() : Double.NaN,
            shooterSpeed);
      }
    }
  }
}
