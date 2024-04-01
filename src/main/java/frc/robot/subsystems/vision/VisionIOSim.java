package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.configs.RobotConfig;
import java.util.EnumSet;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOSim implements VisionIO {
  private double lastTimestamp = 0;
  private PhotonPipelineResult lastResult = new PhotonPipelineResult();

  private final Supplier<Pose2d> poseSupplier;

  private final AprilTagFieldLayout layout;

  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final VisionSystemSim visionSim;

  public VisionIOSim(
      RobotConfig config,
      Supplier<Pose2d> poseSupplier,
      Transform3d robotToCamera,
      String networkName) {
    this.poseSupplier = poseSupplier;
    this.layout = config.getAprilTagFieldLayout();

    Vision.logConfigTagAmount.info(config.getAprilTagFieldLayout().getTags().size());

    var cameraProp = new SimCameraProperties();
    // FIXME: get these values for the cameras we use
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(128.2));
    cameraProp.setCalibError(0.01, 0.01);
    cameraProp.setFPS(25);
    cameraProp.setAvgLatencyMs(25);
    cameraProp.setLatencyStdDevMs(10);

    this.camera = new PhotonCamera(networkName);
    this.visionSim = new VisionSystemSim(networkName);
    visionSim.addAprilTags(layout);
    this.cameraSim = new PhotonCameraSim(camera, cameraProp);

    visionSim.addCamera(cameraSim, robotToCamera);

    cameraSim.enableDrawWireframe(true);
    cameraSim.enableProcessedStream(true);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    /*
       * based on
    https://docs.wpilib.org/en/latest/docs/software/networktables/listening-for-change.html#listening-for-changes
       * and
    https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/subsystems/vision
    * /VisionIOPhotonVision.java
       */
    DoubleArraySubscriber targetPoseSub =
        inst.getTable("/photonvision/" + networkName)
            .getDoubleArrayTopic("targetPose")
            .subscribe(new double[0]);

    inst.addListener(
        targetPoseSub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          // FIXME Use CTRE TIME check what sim module does
          PhotonPipelineResult result = camera.getLatestResult();
          double timestamp = Utils.getCurrentTimeSeconds() - (result.getLatencyMillis() / 1000.0);
          synchronized (VisionIOSim.this) {
            lastTimestamp = timestamp;
            lastResult = result;
          }
        });
  }

  @Override
  public synchronized void updateInputs(Inputs inputs) {
    visionSim.update(poseSupplier.get());

    inputs.lastTimestampCTRETime = this.lastTimestamp;
    inputs.lastResult = this.lastResult;
    inputs.connected = camera.isConnected();
  }

  @Override
  public PhotonCamera getCamera() {
    return camera;
  }
}
