package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOReal implements ShooterIO {
  TalonFX launcher_lead = new TalonFX(Constants.CanIDs.SHOOTER_LEAD_CAN_ID);
  TalonFX launcher_follower = new TalonFX(Constants.CanIDs.SHOOTER_FOLLOW_CAN_ID);
  TalonFX pivot = new TalonFX(Constants.CanIDs.SHOOTER_PIVOT_CAN_ID);
  TalonFX kicker = new TalonFX(Constants.CanIDs.SHOOTER_KICKER_CAN_ID);

  public Rotation2d pitch = new Rotation2d();

  private final StatusSignal<Double> pivotPosition;
  private final StatusSignal<Double> pivotVelocity;
  private final StatusSignal<Double> pivotVoltage;
  private final StatusSignal<Double> pivotCurrentAmps;

  private final StatusSignal<Double> launcherLeadVelocity;
  private final StatusSignal<Double> launcherLeadVoltage;
  private final StatusSignal<Double> launcherFollowerVoltage;
  private final StatusSignal<Double> launcherLeadCurrentAmps;
  private final StatusSignal<Double> launcherFollowerCurrentAmps;

  private final StatusSignal<Double> kickerAppliedVolts;
  private final StatusSignal<Double> kickerCurrentAmps;
  private final StatusSignal<Double> kickerRPS;

  private final StatusSignal<Double> launcherLeadTempCelsius;
  private final StatusSignal<Double> launcherFollowTempCelsius;
  private final StatusSignal<Double> pivotTempCelsius;
  private final StatusSignal<Double> kickerTempCelsius;

  // FIX: add FOC
  private final MotionMagicVoltage pivotClosedLoopControl =
      new MotionMagicVoltage(0).withEnableFOC(false);
  private final VoltageOut pivotOpenLoop = new VoltageOut(0.0).withEnableFOC(false);

  private final MotionMagicVelocityVoltage launcherClosedLoop =
      new MotionMagicVelocityVoltage(0.0).withEnableFOC(false);
  private final VoltageOut launcherOpenLoop = new VoltageOut(0.0).withEnableFOC(false);

  private final VoltageOut kickerOpenLoop = new VoltageOut(0.0).withEnableFOC(false);

  public ShooterIOReal() {
    // --- launcher config ---
    TalonFXConfiguration launcherConfig = new TalonFXConfiguration();

    // FIXME: get true current limits
    launcherConfig.CurrentLimits.SupplyCurrentLimit = 40;
    launcherConfig.CurrentLimits.SupplyCurrentThreshold = 60.0;
    launcherConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    launcherConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    launcherConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    launcherConfig.Feedback.SensorToMechanismRatio = ShooterConstants.Launcher.GEARING;

    launcher_lead.getConfigurator().apply(launcherConfig);
    launcher_follower.getConfigurator().apply(launcherConfig);
    launcher_follower.setControl(new Follower(Constants.CanIDs.SHOOTER_LEAD_CAN_ID, false));

    // --- pivot config ---
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    pivotConfig.CurrentLimits.SupplyCurrentLimit = 40;
    pivotConfig.CurrentLimits.SupplyCurrentThreshold = 60.0;
    pivotConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    pivotConfig.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.Pivot.GEARING;
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivot.getConfigurator().apply(pivotConfig);

    // -- kicker config --
    TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

    kickerConfig.CurrentLimits.SupplyCurrentLimit = 40;
    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    kicker.getConfigurator().apply(pivotConfig);

    pivotPosition = pivot.getPosition();
    pivotVelocity = pivot.getVelocity();
    pivotVoltage = pivot.getMotorVoltage();
    pivotCurrentAmps = pivot.getStatorCurrent();

    launcherLeadVelocity = launcher_lead.getVelocity();

    launcherLeadVoltage = launcher_lead.getMotorVoltage();
    launcherFollowerVoltage = launcher_follower.getMotorVoltage();
    launcherLeadCurrentAmps = launcher_lead.getStatorCurrent();
    launcherFollowerCurrentAmps = launcher_follower.getStatorCurrent();

    kickerAppliedVolts = kicker.getMotorVoltage();
    kickerCurrentAmps = kicker.getStatorCurrent();
    kickerRPS = kicker.getRotorVelocity();

    launcherLeadTempCelsius = launcher_lead.getDeviceTemp();
    launcherFollowTempCelsius = launcher_lead.getDeviceTemp();
    pivotTempCelsius = pivot.getDeviceTemp();
    kickerTempCelsius = kicker.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100, pivotPosition, launcherLeadVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, pivotVelocity, pivotVoltage, pivotCurrentAmps, kickerAppliedVolts, kickerCurrentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(
        10, launcherFollowerVoltage, launcherFollowerCurrentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(
        1, launcherLeadTempCelsius, launcherFollowTempCelsius, kickerTempCelsius, pivotTempCelsius);

    launcher_lead.optimizeBusUtilization();
    launcher_follower.optimizeBusUtilization();
    pivot.optimizeBusUtilization();
    kicker.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotPosition,
        pivotVelocity,
        pivotVoltage,
        pivotCurrentAmps,
        // --
        launcherLeadVelocity,
        launcherLeadVoltage,
        launcherFollowerVoltage,
        launcherLeadCurrentAmps,
        launcherFollowerCurrentAmps,
        // --
        kickerAppliedVolts,
        kickerCurrentAmps,
        kickerRPS,
        // --
        launcherLeadTempCelsius,
        launcherFollowTempCelsius,
        pivotTempCelsius,
        kickerTempCelsius);

    inputs.pivotPosition = Rotation2d.fromRotations(pivotPosition.getValueAsDouble());
    // rotations to rads = mult by 2pi. 1 full rot = 2pi rads
    inputs.pivotVelocityRadsPerSec = pivotVelocity.getValueAsDouble() * (2 * Math.PI);
    inputs.pivotAppliedVotls = pivotVoltage.getValueAsDouble();
    inputs.pivotCurrentAmps = pivotCurrentAmps.getValueAsDouble();

    inputs.launcherRPM = launcherLeadVelocity.getValueAsDouble() * 60; // rps to rpm = mult by 60
    inputs.launcherAppliedVolts =
        new double[] {
          launcherLeadVoltage.getValueAsDouble(), launcherFollowerVoltage.getValueAsDouble()
        };
    inputs.launcherCurrentAmps =
        new double[] {
          launcherLeadCurrentAmps.getValueAsDouble(), launcherFollowerCurrentAmps.getValueAsDouble()
        };

    inputs.kickerAppliedVolts = kickerAppliedVolts.getValueAsDouble();
    inputs.kickerCurrentAmps = kickerCurrentAmps.getValueAsDouble();
    inputs.kickerRPM = kickerRPS.getValueAsDouble() * 60.0;

    inputs.tempsCelcius =
        new double[] {
          launcherLeadTempCelsius.getValueAsDouble(),
          launcherFollowTempCelsius.getValueAsDouble(),
          pivotTempCelsius.getValueAsDouble(),
          kickerTempCelsius.getValueAsDouble()
        };
  }

  // PIVOT

  @Override
  public void setPivotPosition(Rotation2d rot) {
    pivotClosedLoopControl.withPosition(rot.getRotations());
    pivot.setControl(pivotClosedLoopControl);
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivot.setControl(pivotOpenLoop.withOutput(volts));
  }

  // LAUNCHER
  @Override
  public void setLauncherVoltage(double volts) {
    launcher_lead.setControl(launcherOpenLoop.withOutput(volts));
  }

  @Override
  public void setLauncherRPM(double rpm) {
    launcher_lead.setControl(launcherClosedLoop.withVelocity(rpm));
  }

  @Override
  public void setKickerVoltage(double volts) {
    kicker.setControl(kickerOpenLoop.withOutput(volts));
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
