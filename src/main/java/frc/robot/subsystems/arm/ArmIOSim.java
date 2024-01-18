package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private double armGearing = 6.0;
  private double momentOfInertia = 1.4;
  private double armLengthMeters = Units.inchesToMeters(12);

  private double minAngleRad = Units.degreesToRadians(270);
  private double maxAngleRad = Units.degreesToRadians(90);

  SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500Foc(1),
          armGearing,
          momentOfInertia,
          armLengthMeters,
          minAngleRad,
          maxAngleRad,
          false,
          0);

  private final ProfiledPIDController feedback;

  private boolean closedLoop = false;
  private Rotation2d closedLoopTargetAngle = new Rotation2d();
  private double openLoopVolts = 0.0;

  public ArmIOSim() {
    feedback = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    if (closedLoop) {
      var controlEffort =
          feedback.calculate(inputs.armPositionRad, closedLoopTargetAngle.getRadians());

      armSim.setInputVoltage(controlEffort);
    } else {
      armSim.setInputVoltage(openLoopVolts);
    }

    armSim.update(0.02);

    inputs.armPositionRad = armSim.getAngleRads();
    inputs.armAppliedVolts = armSim.getOutput(0);
    inputs.armCurrentAmps = armSim.getCurrentDrawAmps();
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
  }

  @Override
  public void setClosedLoopConstants(
      double kP, double kD, double maxProfiledVelocity, double maxProfiledAcceleration) {

    feedback.setP(kP);
    feedback.setD(kD);
    feedback.setConstraints(new Constraints(maxProfiledVelocity, maxProfiledAcceleration));
  }
}
