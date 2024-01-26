package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ArmIOSim implements ArmIO {
  private double MOMENT_OF_INERTIA = 0.15;
  private double ARM_LENGTH_METERS = Units.inchesToMeters(14);

  SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500Foc(1),
          Constants.ArmConstants.GEAR_RATIO,
          MOMENT_OF_INERTIA,
          ARM_LENGTH_METERS,
          Constants.ArmConstants.MIN_ARM_ANGLE.getRadians(),
          Constants.ArmConstants.MAX_ARM_ANGLE.getRadians(),
          true,
          0);

  private final ProfiledPIDController feedback;

  private double kG;

  private boolean closedLoop = false;
  private Rotation2d closedLoopTargetAngle = new Rotation2d();
  private double openLoopVolts = 0.0;

  public ArmIOSim() {
    feedback = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    var controlEffort = 0.0;

    if (closedLoop) {
      var fb =
          feedback.calculate(inputs.armPosition.getRadians(), closedLoopTargetAngle.getRadians());
      var ff = kG * Math.cos(armSim.getAngleRads());

      controlEffort = fb + ff;

      Logger.recordOutput("Arm/SIM_FF_fG", ff);
      Logger.recordOutput("Arm/SIM_error", feedback.getPositionError());

    } else {
      controlEffort = openLoopVolts;
    }

    controlEffort = MathUtil.clamp(controlEffort, -12, 12);

    armSim.setInputVoltage(controlEffort);
    armSim.update(0.02);

    inputs.armPosition = Rotation2d.fromRadians(armSim.getAngleRads());
    inputs.armAppliedVolts = controlEffort;
    inputs.armCurrentAmps = armSim.getCurrentDrawAmps();

    Logger.recordOutput("Arm/SIM_controlEffort", controlEffort);
  }

  @Override
  public void setVoltage(double volts) {
    openLoopVolts = volts;
    closedLoop = false;
  }

  @Override
  public void setClosedLoopPosition(Rotation2d angle) {
    closedLoop = true;
    closedLoopTargetAngle = angle;
    openLoopVolts = 0.0;
  }

  @Override
  public void setClosedLoopConstants(
      double kP, double kD, double kG, double maxProfiledVelocity, double maxProfiledAcceleration) {

    this.kG = kG;
    feedback.setP(kP);
    feedback.setD(kD);
    feedback.setConstraints(new Constraints(maxProfiledVelocity, maxProfiledAcceleration));
  }

  @Override
  public void resetSensorPosition(Rotation2d angle) {
    armSim.setState(angle.getRadians(), 0.0);
  }
}
