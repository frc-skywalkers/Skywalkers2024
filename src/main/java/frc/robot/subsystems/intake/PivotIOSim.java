package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {
  private SingleJointedArmSim pivot =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1), 45.0 * 44.0 / 18.0, 0.05, 0.25, 0, Math.PI, true, 0);
  private PIDController pid =
      new PIDController(0.0, 0.0, 0.0); // gets configured based on constants in Arm.java

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(PivotIOInputs inputs) {

    pivot.update(0.02);
    inputs.positionRad = pivot.getAngleRads();
    inputs.velocityRadPerSec = pivot.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = pivot.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    volts = MathUtil.clamp(volts, -6, 6);
    appliedVolts = volts;
    pivot.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  public void reset() {}
  
  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
