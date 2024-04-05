package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class ArmIOReal implements ArmIO {
  private final StatusSignal<Double> appliedVols;
  private final StatusSignal<Double> positionRotations;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> tempCelsius;
  private final StatusSignal<Double> velocity;

  // FIXME: add FOC
  private final MotionMagicVoltage closedLoopControl =
      new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VoltageOut openLoopControl = new VoltageOut(0.0).withEnableFOC(true);

  private final TalonFX motor;

  private final BaseStatusSignal[] refreshSet;

  public ArmIOReal() {
    motor = new TalonFX(Constants.CanIDs.ARM_CAN_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // FIXME: find true current limits
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.ArmConstants.MAX_ARM_ANGLE.getRotations();
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.ArmConstants.MIN_ARM_ANGLE.minus(Rotation2d.fromDegrees(2.0)).getRotations();

    config.Feedback.SensorToMechanismRatio = Constants.ArmConstants.GEAR_RATIO;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Voltage.SupplyVoltageTimeConstant = 0.02;
    // other closed loop configuration is handled by setClosedLoopConstants()

    motor.getConfigurator().apply(config);

    appliedVols = motor.getMotorVoltage();
    positionRotations = motor.getPosition();
    currentAmps = motor.getStatorCurrent();
    tempCelsius = motor.getDeviceTemp();
    velocity = motor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(100, appliedVols, positionRotations, velocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50, currentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(1, tempCelsius);

    motor.optimizeBusUtilization();

    refreshSet =
        new BaseStatusSignal[] {appliedVols, positionRotations, currentAmps, tempCelsius, velocity};
  }

  @Override
  public void updateInputs(Inputs inputs) {
    BaseStatusSignal.refreshAll(refreshSet);

    // could look into latency compensating this value
    inputs.armPosition = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.armAngleDegrees = inputs.armPosition.getDegrees();
    inputs.armAppliedVolts = appliedVols.getValueAsDouble();
    inputs.armCurrentAmps = currentAmps.getValueAsDouble();
    inputs.armTempCelsius = tempCelsius.getValueAsDouble();
    inputs.armVelocity = velocity.getValueAsDouble();
  }

  @Override
  public void setClosedLoopPosition(Rotation2d angle) {
    closedLoopControl.withPosition(angle.getRotations());
    motor.setControl(closedLoopControl);
  }

  @Override
  public void setClosedLoopConstants(
      double kP, double kD, double kG, double maxProfiledVelocity, double maxProfiledAcceleration) {
    var pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    motor.getConfigurator().refresh(pidConfig);
    motor.getConfigurator().refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kD = kD;
    pidConfig.kG = kG;

    mmConfig.MotionMagicCruiseVelocity = maxProfiledVelocity;
    mmConfig.MotionMagicAcceleration = maxProfiledAcceleration;

    motor.getConfigurator().apply(pidConfig);
    motor.getConfigurator().apply(mmConfig);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(openLoopControl.withOutput(volts));
  }

  @Override
  public void resetSensorPosition(Rotation2d angle) {
    motor.setPosition(angle.getRotations());
  }

  @Override
  public void setNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status = motor.getConfigurator().refresh(config);

    if (status != StatusCode.OK) return;

    config.NeutralMode = value;

    motor.getConfigurator().apply(config);
  }
}
