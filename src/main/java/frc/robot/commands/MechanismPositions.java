package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
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

  public record MechanismPosition(double elevatorHeight, Rotation2d armAngle) {}

  public static MechanismPosition loadingPosition() {
    return new MechanismPosition(
        homeElevatorHeightInches.get(), Rotation2d.fromDegrees(homeArmAngleDegrees.get()));
  }

  public static MechanismPosition ampPosition() {
    return new MechanismPosition(
        ampElevatorHeightInches.get(), Rotation2d.fromDegrees(ampArmAngleDegrees.get()));
  }
}
