package frc.robot.subsystems.visionGamepiece;

import frc.robot.Constants;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionGamepieceIOReal implements VisionGamepieceIO {

  final PhotonCamera camera;

  public VisionGamepieceIOReal() {
    camera = new PhotonCamera(Constants.VisionGamepieceConstants.CAMERA_NAME);
    camera.setDriverMode(false);
    camera.setLED(VisionLEDMode.kOff);
  }

  @Override
  public void updateInputs(VisionGamepieceIOInputs inputs) {
    PhotonPipelineResult results = camera.getLatestResult();
    inputs.isConnected = camera.isConnected();
    inputs.validTarget = results.hasTargets();
    List<PhotonTrackedTarget> targets = results.targets;
    inputs.pitch = new double[targets.size()];
    inputs.yaw = new double[targets.size()];
    for (int index = 0; index < targets.size(); index++) {
      inputs.pitch[index] = targets.get(index).getPitch();
      inputs.yaw[index] = targets.get(index).getYaw();
    }
    inputs.totalLatencyMs = results.getLatencyMillis();
    inputs.timestamp = results.getTimestampSeconds();
    inputs.targetCount = targets.size();
  }
}
