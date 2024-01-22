package frc.robot.mechanismVisualization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class SimpleMechanismVisualization {

  static Mechanism2d armMechanism2d =
      new Mechanism2d(Units.inchesToMeters(32), Units.inchesToMeters(50));

  static MechanismRoot2d armRoot =
      armMechanism2d.getRoot("armRoot", Units.inchesToMeters(32 / 2), Units.inchesToMeters(50 / 2));

  static MechanismLigament2d arm =
      armRoot.append(new MechanismLigament2d("arm", Units.inchesToMeters(12), 270));

  static Mechanism2d shooterMechanism2d =
      new Mechanism2d(Units.inchesToMeters(32.0), Units.inchesToMeters(50.0));

  static MechanismRoot2d shooterRoot =
      shooterMechanism2d.getRoot("shooterRoot", Units.inchesToMeters(27), Units.inchesToMeters(4));

  static MechanismLigament2d shooter =
      shooterRoot.append(
          new MechanismLigament2d(
              "shooter",
              Constants.ShooterConstants.SHOOTER_LENGTH,
              Constants.ShooterConstants.Pivot.SIM_INITIAL_ANGLE));

  public static void updateVisualization(Rotation2d armAngle, Rotation2d shooterAngle) {
    arm.setAngle(armAngle);
    shooter.setAngle(shooterAngle);
  }

  public static void logMechanism() {
    Logger.recordOutput("Mechanism/SimpleArm", armMechanism2d);
    Logger.recordOutput("Mechanism/SimpleShooter", shooterMechanism2d);
  }
}
