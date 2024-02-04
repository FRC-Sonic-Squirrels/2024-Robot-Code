package frc.lib.team2930;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

class ShootingSolverTest {
  @Test()
  void testOrbit() {
    Translation3d Pspeaker = new Translation3d(0, 0, 10);
    Translation3d PaxisOfRotationShooter = new Translation3d(0, 0, 0);
    double shootingTime = 1;

    for (double shooterSpeed = 10; shooterSpeed <= 50; shooterSpeed += 10) {
      var solver = new ShootingSolver(Pspeaker, PaxisOfRotationShooter, shooterSpeed, shootingTime);
      for (double i = -10; i <= 10; i += 0.1) {
        var res =
            solver.computeAngles(0, new Pose2d(-i, 10, new Rotation2d(0)), new Translation2d(i, 0));

        System.out.printf(
            "%3.1f: heading:%3.1f pitch:%3.1f shooter:%2.1f\n",
            i,
            res != null ? res.heading().getDegrees() : Double.NaN,
            res != null ? res.pitch().getDegrees() : Double.NaN,
            shooterSpeed);
      }
    }
  }
}
