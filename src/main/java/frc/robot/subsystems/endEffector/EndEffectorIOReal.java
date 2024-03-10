package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.units.Units;
import frc.lib.team2930.ControlMode;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOReal implements EndEffectorIO {
  private final TalonFX motor = new TalonFX(Constants.CanIDs.END_EFFECTOR_CAN_ID);

  TimeOfFlight intake_tof = new TimeOfFlight(Constants.CanIDs.END_EFFECTOR_INTAKE_SIDE_TOF_CAN_ID);
  TimeOfFlight shooter_tof =
      new TimeOfFlight(Constants.CanIDs.END_EFFECTOR_SHOOTER_SIDE_TOF_CAN_ID);

  private final StatusSignal<Double> deviceTemp;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> velocityRPS;

  private final VoltageOut openLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final VelocityVoltage closedLoopControl = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

  private final BaseStatusSignal[] refreshSet;

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

    shooter_tof.setRangeOfInterest(6, 6, 10, 10);
    intake_tof.setRangeOfInterest(6, 6, 10, 10);

    shooter_tof.setRangingMode(RangingMode.Short, 25);
    intake_tof.setRangingMode(RangingMode.Short, 25);

    deviceTemp = motor.getDeviceTemp();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();
    velocityRPS = motor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(100, appliedVolts);
    BaseStatusSignal.setUpdateFrequencyForAll(50, currentAmps, velocityRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(1, deviceTemp);

    motor.optimizeBusUtilization();

    refreshSet = new BaseStatusSignal[] {deviceTemp, appliedVolts, currentAmps, velocityRPS};
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    BaseStatusSignal.refreshAll(refreshSet);

    inputs.velocityRPM = velocityRPS.getValueAsDouble() * 60.0;
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.tempCelsius = deviceTemp.getValueAsDouble();
    inputs.intakeSideTOFDistanceInches =
        Units.Millimeters.of(intake_tof.getRange()).in(Units.Inches);
    inputs.shooterSideTOFDistanceInches =
        Units.Millimeters.of(shooter_tof.getRange()).in(Units.Inches);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(openLoopControl.withOutput(volts));
  }

  @Override
  public void setVelocity(double revPerMin) {
      motor.setControl(closedLoopControl.withVelocity(revPerMin / 60.0));
  }

  @Override
  public void setClosedLoopConstants(
      double kP, double kV, double kS, double maxProfiledAcceleration) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    var config = motor.getConfigurator();

    config.refresh(pidConfig);
    config.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kV = kV;
    pidConfig.kS = kS;

    mmConfig.MotionMagicAcceleration = maxProfiledAcceleration;

    config.apply(pidConfig);
    config.apply(mmConfig);
  }
}
