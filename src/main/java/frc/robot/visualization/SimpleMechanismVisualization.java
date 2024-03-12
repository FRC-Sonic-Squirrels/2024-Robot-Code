package frc.robot.visualization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;

public class SimpleMechanismVisualization {
  private static final LoggerGroup logGroup = new LoggerGroup("Mechanism");
  private static final LoggerEntry logSimpleElevatorAndArm = logGroup.build("SimpleElevatorAndArm");
  private static final LoggerEntry logSimpleShooter = logGroup.build("SimpleShooter");

  static Mechanism2d shooterMechanism2d =
      new Mechanism2d(
          Units.Inches.of(32.0).in(Units.Meters), Units.Inches.of(50.0).in(Units.Meters));

  static MechanismRoot2d shooterRoot =
      shooterMechanism2d.getRoot(
          "shooterRoot", Units.Inches.of(4).in(Units.Meters), Units.Inches.of(4).in(Units.Meters));

  static MechanismLigament2d shooter =
      shooterRoot.append(
          new MechanismLigament2d(
              "shooter",
              -Constants.ShooterConstants.SHOOTER_LENGTH.in(Units.Meters),
              Constants.ShooterConstants.Pivot.SIM_INITIAL_ANGLE));

  static MechanismLigament2d gamepieceTraj =
      shooterRoot.append(
          new MechanismLigament2d(
              "gamepieceTraj",
              Constants.ShooterConstants.SHOOTER_LENGTH.in(Units.Meters),
              Constants.ShooterConstants.Pivot.SIM_INITIAL_ANGLE));

  static Mechanism2d elevatorAndArmMech2d =
      new Mechanism2d(
          Units.Inches.of(32.0).in(Units.Meters), Units.Inches.of(50.0).in(Units.Meters));

  static MechanismRoot2d elevatorRoot =
      elevatorAndArmMech2d.getRoot(
          "elevatorRoot",
          Units.Inches.of(18).in(Units.Meters),
          Units.Inches.of(4).in(Units.Meters));

  static MechanismLigament2d elevator =
      elevatorRoot.append(new MechanismLigament2d("elevator", 0.0, 90.0));

  // FIXME: get correct numbers from CAD
  static MechanismLigament2d armConnectionToElevator =
      elevator.append(
          new MechanismLigament2d("armConnection", Units.Inches.of(3).in(Units.Meters), 40.0));

  static MechanismLigament2d arm =
      armConnectionToElevator.append(
          new MechanismLigament2d("arm", Units.Inches.of(12).in(Units.Meters), 150.0));

  static {
    elevator.setColor(new Color8Bit(0, 0, 255));
    elevator.setLineWeight(15.0);

    arm.setColor(new Color8Bit(0, 255, 0));
  }

  public static void updateVisualization(
      Rotation2d armAngle,
      Rotation2d shooterAngle,
      double distToSpeaker,
      double shooterRPM,
      double elevatorHeightInches) {
    arm.setAngle(armAngle.plus(Rotation2d.fromDegrees(140)));

    elevator.setLength(Units.Inches.of(elevatorHeightInches).in(Units.Meters));
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
    logSimpleElevatorAndArm.info(elevatorAndArmMech2d);
    logSimpleShooter.info(shooterMechanism2d);
  }
}
