package frc.robot.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class MechanismVisualization {
  private static Pose3d elevatorStage = new Pose3d();
  private static Pose3d elevatorCarriage = new Pose3d();
  private static Pose3d arm = new Pose3d();
  private static Pose3d shooter = new Pose3d();
  private static Pose3d gamepiece = new Pose3d();

  public static void updateVisualization(
      Rotation2d armAngle,
      Rotation2d shooterAngle,
      double elevatorHeightInches,
      boolean endEffectorBeamBreak) {
    elevatorStage =
        new Pose3d(
            0.0,
            0.0,
            Units.Inches.of(
                        elevatorHeightInches
                            - 8.0
                            - 8.0
                                * (Constants.ElevatorConstants.MAX_HEIGHT.in(Units.Inches)
                                    - elevatorHeightInches)
                                / Constants.ElevatorConstants.MAX_HEIGHT.in(Units.Inches))
                    .in(Units.Meters)
                * 12.0
                / Constants.ElevatorConstants.MAX_HEIGHT.in(Units.Inches),
            new Rotation3d());
    elevatorCarriage =
        new Pose3d(
            0.0,
            0.0,
            Units.Inches.of(elevatorHeightInches - 8.0).in(Units.Meters),
            new Rotation3d());
    arm =
        new Pose3d(
            0.24,
            0.0,
            0.535 + elevatorCarriage.getZ(),
            new Rotation3d(0.0, -armAngle.getRadians() + Math.PI, 0.0));
    shooter = new Pose3d(0.025, 0.0, 0.257, new Rotation3d(0.0, shooterAngle.getRadians(), 0.0));
    gamepiece =
        // endEffectorBeamBreak ?
        arm
    // : new Pose3d(100, 100, 100, new Rotation3d())
    ;
  }

  public static void logMechanism() {
    Logger.recordOutput(
        "Visualization/Mechanism",
        new Pose3d[] {elevatorStage, elevatorCarriage, arm, shooter, gamepiece});

    Logger.recordOutput("Visualization/TestPose", new Pose2d());
  }
}
