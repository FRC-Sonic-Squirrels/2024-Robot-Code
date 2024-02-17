package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.lib.team6328.LoggedTunableNumber;

public class MechanismPositions {
  private static LoggedTunableNumber homeElevatorHeightInches =
      new LoggedTunableNumber("MechanismPositions/homeElevatorHeightInches", 0.0);
  private static LoggedTunableNumber homeArmAngleDegrees =
      new LoggedTunableNumber("MechanismPositions/homeArmAngleDegrees", -90.0);

  private static LoggedTunableNumber ampElevatorHeightInches =
      new LoggedTunableNumber("MechanismPositions/ampElevatorHeightInches", 4.0);
  private static LoggedTunableNumber ampArmAngleDegrees =
      new LoggedTunableNumber("MechanismPositions/ampArmAngleDegrees", 50.0);

  public record MechanismPosition(Measure<Distance> elevatorHeight, Rotation2d armAngle) {}

  public static MechanismPosition loadingPosition() {
    return new MechanismPosition(
        Units.Inches.of(homeElevatorHeightInches.get()),
        Rotation2d.fromDegrees(homeArmAngleDegrees.get()));
  }

  public static MechanismPosition ampPosition() {
    return new MechanismPosition(
        Units.Inches.of(ampElevatorHeightInches.get()),
        Rotation2d.fromDegrees(ampArmAngleDegrees.get()));
  }
}
