package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {

  public static class VisionIOInputs implements LoggableInputs {
    PhotonPipelineResult lastResult = new PhotonPipelineResult();
    double lastTimestampCTRETime = -1.0;
    boolean connected = false;
    double medianLatency = 0.0;
    double medianUpdateTime = 0.0;

    @Override
    public void toLog(LogTable table) {
      byte[] photonPacketBytes = new byte[lastResult.getPacketSize()];
      PhotonPipelineResult.serde.pack(new Packet(photonPacketBytes), lastResult);
      table.put("photonPacketBytes", photonPacketBytes);

      table.put("lastTimestampCTRETime", lastTimestampCTRETime);
      table.put("connected", connected);
      table.put("medianLatency", medianLatency);
      table.put("medianUpdateTime", medianUpdateTime);
    }

    @Override
    public void fromLog(LogTable table) {
      byte[] photonPacketBytes = table.get("photonPacketBytes", new byte[0]);
      lastResult = PhotonPipelineResult.serde.unpack(new Packet(photonPacketBytes));

      lastTimestampCTRETime = table.get("lastTimestampCTRETime", -1.0);
      connected = table.get("connected", false);
      medianLatency = table.get("medianLatency", 0.0);
      medianUpdateTime = table.get("medianUpdateTime", 0.0);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  public default PhotonCamera getCamera() {
    System.out.println("------default getCamera() == null  ------------");
    return null;
  }

  public default void updateMedians(double latency, double timeSinceLastUpdate) {
    // medianLatency = latencyMedianFilter.calculate(latency);
    // medianUpdateTime = updateTimeMedianFilter.calculate(timeSinceLastUpdate);
  }
}
