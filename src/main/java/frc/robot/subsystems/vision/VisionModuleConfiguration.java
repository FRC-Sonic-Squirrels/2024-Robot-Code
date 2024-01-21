package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionModuleConfiguration {

  public final VisionIO visionIO;
  public final String logName;
  public final Transform3d robotToCamera;

  public VisionModuleConfiguration(VisionIO visionIO, String logName, Transform3d robotToCamera) {
    this.visionIO = visionIO;
    this.logName = logName;
    this.robotToCamera = robotToCamera;
  }
}
