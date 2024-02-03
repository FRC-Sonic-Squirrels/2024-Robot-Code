package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class EndEffectorIOReal implements EndEffectorIO {
  private TalonFX motor = new TalonFX(Constants.CanIDs.END_EFFECTOR_CAN_ID);

  private StatusSignal<Double> velocityRPS;
  private StatusSignal<Double> deviceTemp;
  private StatusSignal<Double> intakeSideTOFDistanceInches;
  private StatusSignal<Double> shooterSideTOFDistanceInches;

  private double voltage = 0.0;

  public EndEffectorIOReal() {

    motor.setInverted(false);
    motor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();

    currentLimitConfig.SupplyCurrentLimitEnable =
        Constants.EndEffectorConstants.SUPPLY_CURRENT_LIMIT_ENABLE;
    currentLimitConfig.SupplyCurrentThreshold =
        Constants.EndEffectorConstants.SUPPLY_CURRENT_THRESHOLD;
    currentLimitConfig.SupplyCurrentLimit = Constants.EndEffectorConstants.SUPPLY_CURRENT_LIMIT;
    currentLimitConfig.SupplyTimeThreshold = Constants.EndEffectorConstants.SUPPLY_TIME_THRESHOLD;

    config.CurrentLimits = currentLimitConfig;

    motor.getConfigurator().apply(config);

    velocityRPS = motor.getVelocity();
    deviceTemp = motor.getDeviceTemp();
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocityRPS, deviceTemp, intakeSideTOFDistanceInches, shooterSideTOFDistanceInches);

    motor.setControl(new VoltageOut(voltage));

    inputs.RPM = velocityRPS.getValueAsDouble() / 60.0;
    inputs.tempCelsius = deviceTemp.getValueAsDouble();
    inputs.intakeSideTOFDistanceInches = intakeSideTOFDistanceInches.getValueAsDouble();
    inputs.shooterSideTOFDistanceInches = shooterSideTOFDistanceInches.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    voltage = volts;
  }
}
