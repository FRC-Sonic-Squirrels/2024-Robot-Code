package frc.robot.mechanismVisualization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.littletonrobotics.junction.Logger;

public class SimpleMechanismVisualization {

  static Mechanism2d mechanism2d =
      new Mechanism2d(Units.inchesToMeters(32), Units.inchesToMeters(50));

  static MechanismRoot2d root =
      mechanism2d.getRoot("root", Units.inchesToMeters(32 / 2), Units.inchesToMeters(50 / 2));

  static MechanismLigament2d arm =
      root.append(new MechanismLigament2d("arm", Units.inchesToMeters(12), 270));

  public static void updateVisualization(Rotation2d armAngle) {
    arm.setAngle(armAngle);
  }

  public static void logMechanism() {
    Logger.recordOutput("Mechanism/Simple", mechanism2d);
  }
}
