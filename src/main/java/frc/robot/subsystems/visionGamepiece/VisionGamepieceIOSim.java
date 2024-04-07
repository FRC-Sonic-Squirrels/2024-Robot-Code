package frc.robot.subsystems.visionGamepiece;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.configs.RobotConfig;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionGamepieceIOSim implements VisionGamepieceIO {

  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;

  public VisionGamepieceIOSim(RobotConfig config, Supplier<Pose2d> poseSupplier) {

    this.poseSupplier = poseSupplier;
    var cameraProp = new SimCameraProperties();
    // FIXME: get these values for the cameras we use
    // cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(128.2));
    // cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(25);
    cameraProp.setAvgLatencyMs(25);
    cameraProp.setLatencyStdDevMs(10);

    var networkName = "gamepieceCameraSim";
    this.camera = new PhotonCamera(networkName);
    this.visionSim = new VisionSystemSim(networkName);
    visionSim.addAprilTags(config.getAprilTagFieldLayout());
    this.cameraSim = new PhotonCameraSim(camera, cameraProp);

    var asTransform = Constants.VisionGamepieceConstants.GAMEPIECE_CAMERA_POSE.minus(new Pose3d());
    var robotToCamera =
        asTransform.plus(
            new Transform3d(0, 0, Units.Inch.of(20).in(Units.Meter), new Rotation3d()));

    visionSim.addCamera(cameraSim, robotToCamera);

    cameraSim.enableDrawWireframe(true);
    cameraSim.enableProcessedStream(true);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    visionSim.update(this.poseSupplier.get());

    var results = camera.getLatestResult();

    var aprilTagYaw = 0.0;
    var seesStageTags = false;
    for (int i = 0; i < results.getTargets().size(); i++) {
      PhotonTrackedTarget target = results.targets.get(i);
      if (target.getFiducialId() >= 11) {
        aprilTagYaw = target.getYaw();
        seesStageTags = true;
      }
    }

    inputs.aprilTagYaw = aprilTagYaw;
    inputs.seesStageTags = seesStageTags;
  }
}
