package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotTargetMeasurement {
  double timestamp = 0.0;
  Rotation2d targetRot = new Rotation2d();
  boolean upDirection = true;

  public PivotTargetMeasurement(double timestamp, Rotation2d targetRot, boolean upDirection) {
    this.timestamp = timestamp;
    this.targetRot = targetRot;
    this.upDirection = upDirection;
  }
}
