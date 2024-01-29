package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {
  TalonFX lead = new TalonFX(Constants.CanIDs.SHOOTER_LEAD_CAN_ID);
  TalonFX follow = new TalonFX(Constants.CanIDs.SHOOTER_FOLLOW_CAN_ID);
  TalonFX pivot = new TalonFX(Constants.CanIDs.SHOOTER_PIVOT_CAN_ID);

  TalonFXConfiguration config = new TalonFXConfiguration();

  public Rotation2d pitch = new Rotation2d();
  private final StatusSignal<Double> pivotVoltage;
  ;
  private final StatusSignal<Double> launcherLeadTempCelsius;
  private final StatusSignal<Double> launcherFollowTempCelsius;
  private final StatusSignal<Double> launcherVoltage;

  final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

  public ShooterIOReal() {

    // FIXME: get true current limits
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // does this work for the getConfigurator() method?
    follow.setControl(new Follower(Constants.CanIDs.SHOOTER_LEAD_CAN_ID, false));

    pivotVoltage = pivot.getMotorVoltage();
    launcherLeadTempCelsius = lead.getDeviceTemp();
    launcherFollowTempCelsius = follow.getDeviceTemp();
    launcherVoltage = lead.getMotorVoltage();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // FIXME: add other variables: pivotVoltage, pivotVelocity, RPM
    BaseStatusSignal.refreshAll(
        pivotVoltage, launcherLeadTempCelsius, launcherFollowTempCelsius, launcherVoltage);

    // FIXME: figure out what to put as the argument
    // inputs.pitch = new Rotation2d(pivot.getRotorPosition());

    inputs.pivotVoltage = pivotVoltage.getValue();
    inputs.launcherLeadTempCelsius = launcherLeadTempCelsius.getValueAsDouble();
    inputs.launcherFollowTempCelsius = launcherFollowTempCelsius.getValueAsDouble();
    inputs.launcherVoltage = launcherVoltage.getValueAsDouble();
  }

  // PIVOT

  @Override
  public void setPivotPosition(Rotation2d rot) {
    pitch = rot;
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
  public void setPivotVoltage(double volts) {
    pivot.setVoltage(volts);
  }

  // LAUNCHER

  @Override
  public void setLauncherVoltage(double volts) {
    lead.setVoltage(volts);
  }

  @Override
  public void setLauncherPercentOut(double percent) {
    lead.set(percent);
  }

  @Override
  public void setLauncherClosedLoopConstants(
      double kP, double kD, double kG, double maxProfiledVelocity, double maxProfiledAcceleration) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    lead.getConfigurator().refresh(pidConfig);
    lead.getConfigurator().refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kD = kD;
    pidConfig.kG = kG;

    mmConfig.MotionMagicCruiseVelocity = maxProfiledVelocity;
    mmConfig.MotionMagicAcceleration = maxProfiledAcceleration;

    lead.getConfigurator().apply(pidConfig);
    lead.getConfigurator().apply(mmConfig);
  }
}
