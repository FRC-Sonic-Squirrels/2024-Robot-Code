package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {

  public static class VisionIOInputs implements LoggableInputs {
    PhotonPipelineResult lastResult = new PhotonPipelineResult();
    double lastTimestamp = -1.0;
    boolean connected = false;

    @Override
    public void toLog(LogTable table) {
      byte[] photonPacketBytes = new byte[lastResult.getPacketSize()];
      PhotonPipelineResult.serde.pack(new Packet(photonPacketBytes), lastResult);
      table.put("photonPacketBytes", photonPacketBytes);

      table.put("lastTimestamp", lastTimestamp);
      table.put("connected", connected);
    }

    @Override
    public void fromLog(LogTable table) {
      byte[] photonPacketBytes = table.get("photonPacketBytes", new byte[0]);
      lastResult = PhotonPipelineResult.serde.unpack(new Packet(photonPacketBytes));

      lastTimestamp = table.get("lastTimestamp", -1.0);
      connected = table.get("connected", false);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  public default PhotonCamera getCamera() {
    System.out.println("------default getCamera() == null  ------------");
    return null;
  }
}
