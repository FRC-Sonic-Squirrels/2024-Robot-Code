package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOReal implements EndEffectorIO {
  private TalonFX motor = new TalonFX(Constants.CanIDs.END_EFFECTOR_CAN_ID);

  TimeOfFlight intake_tof = new TimeOfFlight(Constants.CanIDs.END_EFFECTOR_INTAKE_SIDE_TOF_CAN_ID);
  TimeOfFlight shooter_tof =
      new TimeOfFlight(Constants.CanIDs.END_EFFECTOR_SHOOTER_SIDE_TOF_CAN_ID);

  private final StatusSignal<Double> deviceTemp;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> velocityRPS;

  private final VoltageOut openLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  public EndEffectorIOReal() {

    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();

    currentLimitConfig.SupplyCurrentLimit = 30.0;
    currentLimitConfig.SupplyCurrentLimitEnable = true;

    config.CurrentLimits = currentLimitConfig;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = EndEffectorConstants.GEARING;

    motor.getConfigurator().apply(config);

    deviceTemp = motor.getDeviceTemp();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();
    velocityRPS = motor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(100, appliedVolts);
    BaseStatusSignal.setUpdateFrequencyForAll(50, currentAmps, velocityRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(1, deviceTemp);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    BaseStatusSignal.refreshAll(deviceTemp, appliedVolts, currentAmps, velocityRPS);

    inputs.velocityRPM = velocityRPS.getValueAsDouble() * 60.0;
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.tempCelsius = deviceTemp.getValueAsDouble();
    inputs.intakeSideTOFDistanceInches = intake_tof.getRange();
    inputs.shooterSideTOFDistanceInches = shooter_tof.getRange();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(openLoopControl.withOutput(volts));
  }
}
