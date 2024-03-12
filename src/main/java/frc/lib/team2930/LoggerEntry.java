package frc.lib.team2930;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.protobuf.Protobuf;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import us.hebi.quickbuf.ProtoMessage;

public class LoggerEntry {
  public final String key;

  public LoggerEntry(String key) {
    this.key = key;
  }

  public void info(byte[] value) {
    Logger.recordOutput(key, value);
  }

  public void info(boolean value) {
    Logger.recordOutput(key, value);
  }

  public void info(int value) {
    Logger.recordOutput(key, value);
  }

  public void info(long value) {
    Logger.recordOutput(key, value);
  }

  public void info(float value) {
    Logger.recordOutput(key, value);
  }

  public void info(double value) {
    Logger.recordOutput(key, value);
  }

  public void info(String value) {
    Logger.recordOutput(key, value);
  }

  public <E extends Enum<E>> void info(E value) {
    Logger.recordOutput(key, value);
  }

  public <U extends Unit<U>> void info(Measure<U> value) {
    Logger.recordOutput(key, value);
  }

  public void info(boolean[] value) {
    Logger.recordOutput(key, value);
  }

  public void info(int[] value) {
    Logger.recordOutput(key, value);
  }

  public void info(long[] value) {
    Logger.recordOutput(key, value);
  }

  public void info(float[] value) {
    Logger.recordOutput(key, value);
  }

  public void info(double[] value) {
    Logger.recordOutput(key, value);
  }

  public void info(String[] value) {
    Logger.recordOutput(key, value);
  }

  public <T> void info(Struct<T> struct, T value) {
    Logger.recordOutput(key, struct, value);
  }

  public <T> void info(Struct<T> struct, T... value) {
    Logger.recordOutput(key, struct, value);
  }

  public <T, MessageType extends ProtoMessage<?>> void info(
      Protobuf<T, MessageType> proto, T value) {
    Logger.recordOutput(key, proto, value);
  }

  public <T extends WPISerializable> void info(T value) {
    Logger.recordOutput(key, value);
  }

  public <T extends StructSerializable> void info(T... value) {
    Logger.recordOutput(key, value);
  }

  public void info(Mechanism2d value) {
    Logger.recordOutput(key, value);
  }

  public void info(Rotation2d value) {
    info(value.getDegrees());
  }

  public void info(LoggableInputs inputs) {
    Logger.processInputs(key, inputs);
  }
}
