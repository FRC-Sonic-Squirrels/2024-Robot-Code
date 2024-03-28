package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonVision implements VisionIO {
  private final PhotonCamera camera;

  private double lastTimestampCTRETime = -1;
  private PhotonPipelineResult lastResult = new PhotonPipelineResult();

  public double medianLatency = 0.0;
  MedianFilter latencyMedianFilter = new MedianFilter(10);
  public double medianUpdateTime = 0.0;
  MedianFilter updateTimeMedianFilter = new MedianFilter(10);

  public VisionIOPhotonVision(String cameraName) {
    camera = new PhotonCamera(cameraName);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    camera.setDriverMode(false);
    camera.setLED(VisionLEDMode.kOff);

    /*
       * based on
    https://docs.wpilib.org/en/latest/docs/software/networktables/listening-for-change.html#listening-for-changes
       * and
    https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVision.java
       */
    DoubleArraySubscriber targetPoseSub =
        inst.getTable("/photonvision/" + cameraName)
            .getDoubleArrayTopic("targetPose")
            .subscribe(new double[0]);

    inst.addListener(
        targetPoseSub,
        java.util.EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          PhotonPipelineResult result = camera.getLatestResult();
          var timestamp = result.getTimestampSeconds();
          var fpga = Timer.getFPGATimestamp();
          var ctre = Utils.getCurrentTimeSeconds();

          // we use CTRE time here because drivetrain odometry uses CTRE time NOT FPGA
          timestamp -= fpga;
          timestamp += ctre;

          synchronized (VisionIOPhotonVision.this) {
            updateMedians(result.getLatencyMillis(), timestamp - lastTimestampCTRETime);

            lastTimestampCTRETime = timestamp;
            lastResult = result;
          }
        });
  }

  @Override
  public synchronized void updateInputs(Inputs inputs) {
    inputs.lastTimestampCTRETime = this.lastTimestampCTRETime;
    inputs.lastResult = this.lastResult;
    inputs.connected = camera.isConnected();
    inputs.medianLatency = medianLatency;
    inputs.medianUpdateTime = medianUpdateTime;
  }

  @Override
  public PhotonCamera getCamera() {
    return camera;
  }

  public void updateMedians(double latency, double timeSinceLastUpdate) {
    medianLatency = latencyMedianFilter.calculate(latency);
    medianUpdateTime = updateTimeMedianFilter.calculate(timeSinceLastUpdate);
  }
}
