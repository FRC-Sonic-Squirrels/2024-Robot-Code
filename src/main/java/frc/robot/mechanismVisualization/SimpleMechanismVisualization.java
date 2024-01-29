package frc.robot.mechanismVisualization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class SimpleMechanismVisualization {

  // static Mechanism2d armMechanism2d =
  // new Mechanism2d(Units.inchesToMeters(32), Units.inchesToMeters(50));

  // static MechanismRoot2d armRoot =
  // armMechanism2d.getRoot("armRoot", Units.inchesToMeters(32 / 2),
  // Units.inchesToMeters(50 / 2));

  // static MechanismLigament2d arm =
  // armRoot.append(new MechanismLigament2d("arm", Units.inchesToMeters(12),
  // 270));

  static Mechanism2d shooterMechanism2d =
      new Mechanism2d(Units.inchesToMeters(32.0), Units.inchesToMeters(50.0));

  static MechanismRoot2d shooterRoot =
      shooterMechanism2d.getRoot("shooterRoot", Units.inchesToMeters(4), Units.inchesToMeters(4));

  static MechanismLigament2d shooter =
      shooterRoot.append(
          new MechanismLigament2d(
              "shooter",
              -Constants.ShooterConstants.SHOOTER_LENGTH,
              Constants.ShooterConstants.Pivot.SIM_INITIAL_ANGLE));

  static MechanismLigament2d gamepieceTraj =
      shooterRoot.append(
          new MechanismLigament2d(
              "gamepieceTraj",
              Constants.ShooterConstants.SHOOTER_LENGTH,
              Constants.ShooterConstants.Pivot.SIM_INITIAL_ANGLE));

  static Mechanism2d elevatorMechanism2d =
      new Mechanism2d(Units.inchesToMeters(32.0), Units.inchesToMeters(50.0));

  static MechanismRoot2d elevatorRoot =
      elevatorMechanism2d.getRoot(
          "elevatorRoot", Units.inchesToMeters(18), Units.inchesToMeters(4));

  static MechanismLigament2d elevator =
      elevatorRoot.append(
          new MechanismLigament2d("elevator", 0.659 - Units.inchesToMeters(4), 90.0));

  static MechanismLigament2d arm =
      elevator.append(new MechanismLigament2d("arm", Units.inchesToMeters(12), 180));

  public static void updateVisualization(
      Rotation2d armAngle,
      Rotation2d shooterAngle,
      double distToSpeaker,
      double shooterRPM,
      double elevatorHeightInches) {
    arm.setAngle(armAngle.minus(new Rotation2d(Units.degreesToRadians(90.0))));
    elevator.setLength(
        0.659 + Units.inchesToMeters(elevatorHeightInches) - Units.inchesToMeters(4));
    elevator.setColor(new Color8Bit(0, 0, 255));
    elevator.setLineWeight(15.0);
    shooter.setAngle(new Rotation2d(-shooterAngle.getRadians()));
    int Red;
    int Green;
    if (shooterRPM / Constants.ShooterConstants.SHOOTING_RPM <= 0.5) {
      Red = (int) (shooterRPM / Constants.ShooterConstants.SHOOTING_RPM * 2.0 * 255.0);
      Green = 255;
    } else {
      Green = (int) (255 - shooterRPM / Constants.ShooterConstants.SHOOTING_RPM * 2.0 * 255.0);
      Red = 255;
    }
    shooter.setColor(new Color8Bit(Red, Green, 0));
    gamepieceTraj.setAngle(new Rotation2d(-shooterAngle.getRadians()));
    gamepieceTraj.setLength(-distToSpeaker);
    gamepieceTraj.setLineWeight(5.0);
    gamepieceTraj.setColor(new Color8Bit("#FF8000"));
  }

  public static void logMechanism() {
    Logger.recordOutput("Mechanism/SimpleElevatorAndArm", elevatorMechanism2d);
    Logger.recordOutput("Mechanism/SimpleShooter", shooterMechanism2d);
  }
}
