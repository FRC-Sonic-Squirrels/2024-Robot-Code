package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.configs.RobotConfig;
import java.util.function.Supplier;

public class VisionModuleConfiguration {
  public final VisionIO visionIO;
  public final String logName;
  public final Transform3d robotToCamera;

  private VisionModuleConfiguration(VisionIO visionIO, String logName, Transform3d robotToCamera) {
    this.visionIO = visionIO;
    this.logName = logName;
    this.robotToCamera = robotToCamera;
  }

  public static VisionModuleConfiguration build(String name, Transform3d position) {
    var visionIO = new VisionIOPhotonVision(name);
    return new VisionModuleConfiguration(visionIO, name, position);
  }

  public static VisionModuleConfiguration buildSim(
      String name, Transform3d position, RobotConfig config, Supplier<Pose2d> poseSupplier) {
    var visionIO = new VisionIOSim(config, poseSupplier, position, name);
    return new VisionModuleConfiguration(visionIO, name, position);
  }

  public static VisionModuleConfiguration buildReplayStub(String name, Transform3d position) {
    var visionIO = new VisionIO() {};
    return new VisionModuleConfiguration(visionIO, name, position);
  }
}
