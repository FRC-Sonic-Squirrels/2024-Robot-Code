package frc.lib.team2930;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

public class ShootingSolverTest {
  @Test()
  public void testOrbit() {
    Translation3d Pspeaker = new Translation3d(0, 0, 10);
    Translation3d PaxisOfRotationShooter = new Translation3d(0, 0, 0);
    double shootingTime = 1;

    var robotVel = new Translation2d(10, 0);
    var robotPos = new Translation2d(0, 10);

    //
    // Create a trajectory tangential to the speaker, rotating it around the speaker,
    // testing different shooter speeds.
    //
    for (double shooterSpeed = 10; shooterSpeed < 50; shooterSpeed += 10) {

      var solver = new ShootingSolver(Pspeaker, PaxisOfRotationShooter, shooterSpeed, shootingTime);
      for (double angle = 0; angle < 360; angle += 45) {
        var rot = Rotation2d.fromDegrees(angle);

        var robotPosRotated = robotPos.rotateBy(rot);
        var robotVelRotated = robotVel.rotateBy(rot);

        var robotPosRotatedAndTranslated = robotPosRotated.minus(robotVelRotated);
        var robotPose = new Pose2d(robotPosRotatedAndTranslated, new Rotation2d());

        var res = solver.computeRobotYaw(robotPose, robotVel, new Translation2d());
        double VnoteHorizontal = shooterSpeed * Math.cos(Math.toRadians(45));
        if (VnoteHorizontal < 10) {
          assertNull(res);
        } else {

          assertNotNull(res);
          assertEquals(45.0, res.pitch().getDegrees(), 0.0001);

          var angleDiff = angle - res.heading().getDegrees();
          if (angleDiff > 360) {
            angleDiff -= 360;
          }

          assertEquals(180 - Math.toDegrees(Math.acos(10 / VnoteHorizontal)), angleDiff, 0.0001);
        }
      }
    }
  }

  public void printSpeedSweep() {
    Translation3d Pspeaker = new Translation3d(0, 0, 10);
    Translation3d PaxisOfRotationShooter = new Translation3d(0, 0, 0);
    double shootingTime = 1;

    for (double shooterSpeed = 10; shooterSpeed <= 50; shooterSpeed += 10) {
      var solver = new ShootingSolver(Pspeaker, PaxisOfRotationShooter, shooterSpeed, shootingTime);
      for (double i = -10; i <= 10; i += 0.1) {
        var res =
            solver.computeRobotYaw(
                new Pose2d(),
                new Pose2d(-i, 10, new Rotation2d(0)).getTranslation(),
                new Translation2d(i, 0));

        System.out.printf(
            "%3.1f: ROR:%3.1f heading:%3.1f pitch:%3.1f shooter:%2.1f\n",
            i,
            res != null ? res.angularVel() : Double.NaN,
            res != null ? res.heading().getDegrees() : Double.NaN,
            res != null ? res.pitch().getDegrees() : Double.NaN,
            shooterSpeed);
      }
    }
  }
}
