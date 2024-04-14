package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;

public abstract class BaseInputs {
  private final LoggerEntry.Bool log_connected;
  private final LoggerEntry.Text log_statusCode;
  private final LoggerEntry.Decimal log_lastGoodStatus;

  public boolean connected;
  public String statusCode;
  public double lastGoodStatus;

  public BaseInputs(LoggerGroup logInputs) {
    log_connected = logInputs.buildBoolean("Connected");
    log_statusCode = logInputs.buildString("StatusCode");
    log_lastGoodStatus = logInputs.buildDecimal("LastGoodStatus");
  }

  public void refreshAll(BaseStatusSignal[] refreshSet) {
    var code = BaseStatusSignal.refreshAll(refreshSet);
    statusCode = code.getDescription();
    connected = code == StatusCode.OK;
    if (connected) {
      lastGoodStatus = LoggerGroup.getCurrentTimestmap();
      log_lastGoodStatus.info(lastGoodStatus);
    }

    log_connected.info(connected);
    log_statusCode.info(statusCode);
  }

  public boolean wasUpdatedRecently(double maxStale) {
    return lastGoodStatus + maxStale >= LoggerGroup.getCurrentTimestmap();
  }
}
