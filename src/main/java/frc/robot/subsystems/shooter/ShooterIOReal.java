package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {
  TalonFX launcher_lead = new TalonFX(Constants.CanIDs.SHOOTER_LEAD_CAN_ID);
  TalonFX launcher_follower = new TalonFX(Constants.CanIDs.SHOOTER_FOLLOW_CAN_ID);
  TalonFX pivot = new TalonFX(Constants.CanIDs.SHOOTER_PIVOT_CAN_ID);
  TalonFX kicker = new TalonFX(Constants.CanIDs.SHOOTER_KICKER_CAN_ID);

  public Rotation2d pitch = new Rotation2d();

  private final StatusSignal<Double> pivotVoltage;
  private final StatusSignal<Double> pivotRotations;
  private final StatusSignal<Double> launcherLeadTempCelsius;
  private final StatusSignal<Double> launcherFollowTempCelsius;
  private final StatusSignal<Double> launcherVoltage;

  // FIX: add FOC
  private final MotionMagicVoltage closedLoopControl =
      new MotionMagicVoltage(0).withEnableFOC(false);

  private final MotionMagicVelocityVoltage launcherClosedLoop =
      new MotionMagicVelocityVoltage(0.0).withEnableFOC(false);

  public ShooterIOReal() {
    TalonFXConfiguration launcherConfig = new TalonFXConfiguration();

    // FIXME: get true current limits
    launcherConfig.CurrentLimits.SupplyCurrentLimit = 40;
    launcherConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    launcher_follower.setControl(new Follower(Constants.CanIDs.SHOOTER_LEAD_CAN_ID, false));

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    pivotConfig.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.Pivot.GEARING;
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotVoltage = pivot.getMotorVoltage();
    pivotRotations = pivot.getPosition();
    launcherLeadTempCelsius = launcher_lead.getDeviceTemp();
    launcherFollowTempCelsius = launcher_lead.getDeviceTemp();
    launcherVoltage = launcher_lead.getMotorVoltage();

    // FIXME: add BaseStatusSignal.setUpdateFrequency()
    // FIXME: optimize bus utilization
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotVoltage,
        pivotRotations,
        launcherLeadTempCelsius,
        launcherFollowTempCelsius,
        launcherVoltage);

    inputs.pitch = Rotation2d.fromRotations(pivotRotations.getValueAsDouble());
    inputs.pivotVoltage = pivotVoltage.getValue();
    inputs.launcherLeadTempCelsius = launcherLeadTempCelsius.getValueAsDouble();
    inputs.launcherFollowTempCelsius = launcherFollowTempCelsius.getValueAsDouble();
    inputs.launcherVoltage = launcherVoltage.getValueAsDouble();
  }

  // PIVOT

  @Override
  public void setPivotPosition(Rotation2d rot) {
    closedLoopControl.withPosition(rot.getRotations());
    pivot.setControl(closedLoopControl);
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivot.setVoltage(volts);
  }

  // LAUNCHER

  @Override
  public void setLauncherVoltage(double volts) {
    launcher_lead.setVoltage(volts);
  }

  @Override
  public void setPivotClosedLoopConstants(
      double kP, double kD, double kG, double maxProfiledVelocity, double maxProfiledAcceleration) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    pivot.getConfigurator().refresh(pidConfig);
    pivot.getConfigurator().refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kD = kD;
    pidConfig.kG = kG;

    mmConfig.MotionMagicCruiseVelocity = maxProfiledVelocity;
    mmConfig.MotionMagicAcceleration = maxProfiledAcceleration;

    pivot.getConfigurator().apply(pidConfig);
    pivot.getConfigurator().apply(mmConfig);
  }

  @Override
  public void setLauncherClosedLoopConstants(
      double kP, double kD, double maxProfiledVelocity, double maxProfiledAcceleration) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    launcher_lead.getConfigurator().refresh(pidConfig);
    launcher_lead.getConfigurator().refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kD = kD;

    mmConfig.MotionMagicCruiseVelocity = maxProfiledVelocity;
    mmConfig.MotionMagicAcceleration = maxProfiledAcceleration;

    launcher_lead.getConfigurator().apply(pidConfig);
    launcher_lead.getConfigurator().apply(mmConfig);
  }
}
