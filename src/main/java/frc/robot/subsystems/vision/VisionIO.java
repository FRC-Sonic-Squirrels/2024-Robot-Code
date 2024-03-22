package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
  class Inputs {
    PhotonPipelineResult lastResult = new PhotonPipelineResult();
    double lastTimestampCTRETime = -1.0;
    boolean connected = false;
    double medianLatency = 0.0;
    double medianUpdateTime = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Inputs inputs) {}

  public default PhotonCamera getCamera() {
    System.out.println("------default getCamera() == null  ------------");
    return null;
  }

  public default void updateMedians(double latency, double timeSinceLastUpdate) {
    // medianLatency = latencyMedianFilter.calculate(latency);
    // medianUpdateTime = updateTimeMedianFilter.calculate(timeSinceLastUpdate);
  }
}
