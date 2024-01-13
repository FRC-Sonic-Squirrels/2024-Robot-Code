package frc.robot.subsystems.limelight;

public class RawGamepieceData {
  public String className;

  public double classID;

  public double confidence;

  public double ta;

  public double tx;

  public double tx_pixels;

  public double ty;

  public double ty_pixels;

  public double timestamp_RIOFPGA_capture;

  public RawGamepieceData(
      String ClassName,
      double classID,
      double confidence,
      double ta,
      double tx,
      double tx_pixels,
      double ty,
      double ty_pixels,
      double timestamp_RIOFPGA_capture) {
    this.className = className;
    this.classID = classID;
    this.confidence = confidence;
    this.ta = ta;
    this.tx = tx;
    this.tx_pixels = tx_pixels;
    this.ty = ty;
    this.ty_pixels = ty_pixels;
    this.timestamp_RIOFPGA_capture = timestamp_RIOFPGA_capture;
  }
}
