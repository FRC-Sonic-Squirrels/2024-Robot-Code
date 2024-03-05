package frc.lib.team2930;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class PIDTargetMeasurement {
  public double timestamp = 0.0;
  public Rotation2d targetRot = Constants.zeroRotation2d;
  public double target = 0.0;
  public boolean upDirection = true;

  public PIDTargetMeasurement(double timestamp, Rotation2d targetRot, boolean upDirection) {
    this.timestamp = timestamp;
    this.targetRot = targetRot;
    this.upDirection = upDirection;
  }

  public PIDTargetMeasurement(double timestamp, double target, boolean upDirection) {
    this.timestamp = timestamp;
    this.target = target;
    this.upDirection = upDirection;
  }
}
