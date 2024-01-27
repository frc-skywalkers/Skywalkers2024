package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeio;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  private final Pivot pivot;
  public boolean pivotenabled;
  public boolean atGoal;

  /** Creates a new Flywheel. */
  public Intake(IntakeIO intakeio, Pivot pivot) {
    this.intakeio = intakeio;
    this.pivot = pivot;
    this.pivotenabled = pivot.pivotenabled;
    atGoal = pivot.atGoal;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0); // need to determine
        intakeio.configurePID(0.5, 0.0, 0.0); // need to determine
        break;
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        intakeio.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        intakeio.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    intakeio.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/PivotPosition", pivot.getPosition());
    Logger.recordOutput("Intake/PivotSpeed", pivot.getVelocityRPM());
    Logger.recordOutput("Intake/PivotEnabled?", pivot.pivotenabled);
    Logger.recordOutput("Intake/goal", pivot.pivotgoal);
    Logger.recordOutput("Intake/atGoal", pivot.pivotpid.atGoal());
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    intakeio.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    intakeio.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Intake/SetpointRPM", velocityRPM);
  }

  public void pivotRunVelocity(double velocityRPM) {
    pivot.runVelocity(velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    intakeio.stop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public void goToPosition(double goal) {
    pivot.pivotenabled = true;
    pivot.pivotgoal = goal;
    if (pivot.atGoal) {
      pivot.pivotenabled = false;
    }
  }
}
