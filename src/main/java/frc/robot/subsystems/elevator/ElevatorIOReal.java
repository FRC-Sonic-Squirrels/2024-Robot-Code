package frc.robot.subsystems.elevator;

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
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {

  private final TalonFX motor = new TalonFX(Constants.CanIDs.ELEVATOR_CAN_ID);
  private final CANSparkMax reactionArmMotor =
      new CANSparkMax(Constants.CanIDs.REACTION_ARM_CAN_ID, MotorType.kBrushless);

  private SparkPIDController reactionArmPIDController = reactionArmMotor.getPIDController();
  private RelativeEncoder reactionArmEncoder = reactionArmMotor.getEncoder();

  private static final double inchesToMotorRot =
      Constants.ElevatorConstants.GEAR_RATIO
          / (Math.PI * Constants.ElevatorConstants.PULLEY_DIAMETER);

  private final MotionMagicVoltage closedLoopControl =
      new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VoltageOut openLoopControl = new VoltageOut(0.0).withEnableFOC(true);

  private StatusSignal<Double> rotorPosition;
  private StatusSignal<Double> rotorVelocity;
  private StatusSignal<Double> appliedVolts;
  private StatusSignal<Double> currentAmps;
  private StatusSignal<Double> tempCelsius;

  private Servo leftServo = new Servo(0);
  private Servo rightServo = new Servo(1);

  private final BaseStatusSignal[] refreshSet;

  public ElevatorIOReal() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.MAX_HEIGHT.in(Units.Inches) * inchesToMotorRot;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0 * inchesToMotorRot;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    config.Voltage.SupplyVoltageTimeConstant = 0.02;

    motor.getConfigurator().apply(config);

    rotorPosition = motor.getRotorPosition();
    rotorVelocity = motor.getRotorVelocity();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();
    tempCelsius = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100, rotorPosition, rotorVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVolts, currentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(1, tempCelsius);

    motor.optimizeBusUtilization();

    refreshSet =
        new BaseStatusSignal[] {
          rotorPosition, rotorVelocity, appliedVolts, currentAmps, tempCelsius
        };

    reactionArmPIDController.setP(0.15);
    reactionArmPIDController.setOutputRange(-8, 8);
    reactionArmMotor.setSmartCurrentLimit(20);
    reactionArmMotor.setSoftLimit(
        SoftLimitDirection.kForward,
        (float) Constants.ElevatorConstants.ReactionArmConstants.REACTION_ARM_DEPLOY_ROTATIONS
            + (float) 0.5);
    reactionArmMotor.setSoftLimit(
        SoftLimitDirection.kReverse,
        (float) Constants.ElevatorConstants.ReactionArmConstants.REACTION_ARM_HOME_ROTATIONS
            - (float) 0.5);
    reactionArmMotor.burnFlash();

    resetReactionArmPosition();
  }

  @Override
  public void updateInputs(Inputs inputs) {
    BaseStatusSignal.refreshAll(refreshSet);

    inputs.heightInches = rotorPosition.getValueAsDouble() / inchesToMotorRot;
    inputs.velocityInchesPerSecond = rotorVelocity.getValueAsDouble() / inchesToMotorRot;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
    inputs.reactionArmRotations = reactionArmEncoder.getPosition();
    inputs.reactionArmVoltage = reactionArmMotor.getBusVoltage();
  }

  @Override
  public void setVoltage(double volts) {
    openLoopControl.withOutput(volts);
    motor.setControl(openLoopControl);
  }

  @Override
  public void setHeight(Measure<Distance> height) {
    closedLoopControl.withPosition(height.in(Units.Inches) * inchesToMotorRot);
    motor.setControl(closedLoopControl);
  }

  @Override
  public void setSensorPosition(Measure<Distance> position) {
    motor.setPosition(position.in(Units.Inches) * inchesToMotorRot);
  }

  @Override
  public void setPIDConstraints(double kP, double kD, double kG, Constraints constraints) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    motor.getConfigurator().refresh(pidConfig);
    motor.getConfigurator().refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kD = kD;
    pidConfig.kG = kG;

    mmConfig.MotionMagicCruiseVelocity = constraints.maxVelocity;
    mmConfig.MotionMagicAcceleration = constraints.maxAcceleration;

    motor.getConfigurator().apply(pidConfig);
    motor.getConfigurator().apply(mmConfig);
  }

  @Override
  public boolean setNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status = motor.getConfigurator().refresh(config);

    if (status != StatusCode.OK) return false;

    config.NeutralMode = value;

    motor.getConfigurator().apply(config);
    return true;
  }

  @Override
  public void setLeftServoAngle(Rotation2d angle) {
    leftServo.setAngle(angle.getDegrees());
  }

  @Override
  public void setRightServoAngle(Rotation2d angle) {
    rightServo.setAngle(angle.getDegrees());
  }

  @Override
  public void setReactionArmPosition(double rotations) {
    reactionArmPIDController.setReference(rotations, ControlType.kPosition);
  }

  @Override
  public void resetReactionArmPosition() {
    reactionArmEncoder.setPosition(0.0);
  }

  @Override
  public boolean setReactionArmIdleMode(IdleMode idleMode) {
    REVLibError errorCode = reactionArmMotor.setIdleMode(idleMode);
    return errorCode == REVLibError.kOk;
  }
}
