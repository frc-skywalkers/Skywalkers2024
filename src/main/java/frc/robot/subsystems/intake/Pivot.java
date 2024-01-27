package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModelUp;
  private final SimpleMotorFeedforward ffModelDown;
  public final ProfiledPIDController pivotpid;
  public boolean pivotenabled;
  public double pivotgoal;
  public boolean atGoal;

  /** Creates a new Elevator. */
  public Pivot(PivotIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants
        .currentMode) { // currentMode needs to be changed in Constants depending on situation
      case REAL:
        ffModelUp = new SimpleMotorFeedforward(0.613, 1.95);
        ffModelDown = new SimpleMotorFeedforward(-0.424, 1.92);
        pivotpid = new ProfiledPIDController(0.2, 0, 0, new TrapezoidProfile.Constraints(3, 4));
        break;
      case REPLAY:
        ffModelUp = new SimpleMotorFeedforward(0.613, 1.95);
        ffModelDown = new SimpleMotorFeedforward(-0.424, 1.92);
        pivotpid = new ProfiledPIDController(0.2, 0, 0, new TrapezoidProfile.Constraints(3, 4));
        break;
      case SIM:
        ffModelUp = new SimpleMotorFeedforward(0.613, 1.95);
        ffModelDown = new SimpleMotorFeedforward(-0.5, 1.95);
        pivotpid = new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(3, 4));
        break;
      default:
        ffModelUp = new SimpleMotorFeedforward(0.0, 0.0);
        ffModelDown = new SimpleMotorFeedforward(0.0, 0.0);
        pivotpid = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        break;
    }
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
    // Log elevator speed in RPM

    if (pivotenabled) { // only true while goToPosition is running
      pivotpid.setGoal(new State(pivotgoal, 0));
      pivotpid.setTolerance(0.05, 1);
      double volts;
      double error = Math.abs(pivotgoal - inputs.positionRad);

      if (pivotpid.getSetpoint().velocity
          > 0) { // signs of velocities on the way to the goal to see if elevator is going up or
        // down
        volts =
            pivotpid.calculate(inputs.positionRad)
                + ffModelUp.calculate(pivotpid.getSetpoint().velocity);
      } else {
        volts =
            pivotpid.calculate(inputs.positionRad)
                + ffModelDown.calculate(pivotpid.getSetpoint().velocity);
      }

      if (Math.abs(error) < 0.05) {
        volts = 0;
        pivotenabled = false;
      }

      io.setVoltage(volts);
    }
  }

  // stops arm
  public void stop() {
    io.stop();
  }

  public void runVelocity(double velocity) {
    io.setVoltage(velocity * 12);
  }

  // converts to RPM for logging
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  public double getPosition() {
    return inputs.positionRad;
  }

  public void runCharacterizationVolts(double volts) {
    io.setVoltage(volts);
  }

  // avg velocity in radians/sec
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }
}
