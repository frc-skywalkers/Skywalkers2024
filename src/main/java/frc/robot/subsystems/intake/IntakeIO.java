package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double wheelPositionRad = 0.0;
    public double wheelVelocityRadPerSec = 0.0;
    public double wheelAppliedVolts = 0.0;
    public double goalPos = 0.0;
    public double goalVel = 0.0;
    public double setpointPos = 0.000;
    public double ffVolts = 0.0000;
    public double pidOut = 0.000;
    public double tofDistance = 0.000;
    public double tofSD = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setPosition(double positionRad, double ffVolts) {}

  /** Stop in open loop. */
  public default void stop() {}

  public default void runWheelVolts(double volts) {}

  public default void runWheelVel(double velocity) {}

  public default void stopWheels() {}

  public default void resetPosition() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
