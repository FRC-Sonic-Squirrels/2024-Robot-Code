package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class ArmIOReal implements ArmIO {
  // place holder numbers
  private static final double GEAR_RATIO =
    (72.0 / 12.0)* (42.0/16.0);
  // adapted from
  // https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/closed-loop-requests.html#converting-from-meters
  private static final double OUTPUT_RADS_TO_MOTOR_ROTATIONS = (1 / (2 * Math.PI)) * GEAR_RATIO;
  private static final double OUTPUT_ROTATIONS_TO_OUTPUT_RADS = 2 * Math.PI;

  private final StatusSignal<Double> appliedVols;
  private final StatusSignal<Double> positionRotations;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> tempCelsius;

  //FIXME: add FOC
  private final MotionMagicVoltage closedLoopControl =
      new MotionMagicVoltage(0.0).withEnableFOC(false);
  private final VoltageOut openLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final TalonFX motor;

  public ArmIOReal() {
    motor = new TalonFX(Constants.CanIDs.ARM_CAN_ID, "CANivore");

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentThreshold = 0.0;
    config.CurrentLimits.SupplyTimeThreshold = 0.0;
    config.CurrentLimits.SupplyCurrentLimit = 0.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    // other closed loop configuration is handled by setClosedLoopConstants()

    appliedVols = motor.getMotorVoltage();
    positionRotations = motor.getPosition();
    currentAmps = motor.getStatorCurrent();
    tempCelsius = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100, appliedVols, positionRotations);
    BaseStatusSignal.setUpdateFrequencyForAll(50, currentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(1, tempCelsius);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(appliedVols, positionRotations, currentAmps, tempCelsius);
    // could look into latency compensating this value
    inputs.armPosition =
        Rotation2d.fromRadians(
            positionRotations.getValueAsDouble() * OUTPUT_ROTATIONS_TO_OUTPUT_RADS);

    inputs.armAppliedVolts = appliedVols.getValueAsDouble();
    inputs.armCurrentAmps = currentAmps.getValueAsDouble();
    inputs.armTempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void setClosedLoopPosition(Rotation2d angle) {
    var rotations = angle.getRadians() * OUTPUT_RADS_TO_MOTOR_ROTATIONS;
    closedLoopControl.withPosition(rotations);

    motor.setControl(closedLoopControl);
  }

  @Override
  public void setClosedLoopConstants(
      double kP, double kD, double kG, double maxProfiledVelocity, double maxProfiledAcceleration) {
    Slot0Configs pidConfig = new Slot0Configs();
    pidConfig.kP = kP;
    pidConfig.kD = kD;
    pidConfig.kG = kG;

    MotionMagicConfigs mmConfig = new MotionMagicConfigs();
    mmConfig.MotionMagicCruiseVelocity = maxProfiledAcceleration;
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
    var rotations = angle.getRadians() / OUTPUT_ROTATIONS_TO_OUTPUT_RADS;

    motor.setPosition(rotations);
  }
}
