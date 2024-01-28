package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final ArmFeedforward ffModel;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new ArmFeedforward(0.0, 0.0, 0); // need to determine
        io.configurePID(0.5, 0.0, 0.0); // need to determine
        io.configureWheelPID(0.5, 0.0, 0.0);
        break;
      case REPLAY:
        ffModel = new ArmFeedforward(0.1, 0.05, 0);
        io.configurePID(1.0, 0.0, 0.0);
        io.configureWheelPID(0.5, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new ArmFeedforward(0.01, 0.05, 1.5);
        io.configurePID(6.0, 0.0, 0.0);
        io.configureWheelPID(5.0, 0.0, 0.0);
        break;
      default:
        ffModel = new ArmFeedforward(0.0, 0.0, 0);
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs(("Intake"), inputs);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void runWheelVolts(double volts) {
    io.runWheelVolts(volts);
  }

  public void runWheelVel(double velocity) {
    io.runWheelVel(velocity);
  }

  public void setPosition(double positionRad) {
    if (Constants.currentMode == Mode.SIM) {
      io.setPosition(positionRad, ffModel.calculate(inputs.setpointPos, inputs.goalVel));
    } else {
      io.setPosition(positionRad, ffModel.calculate(inputs.positionRad, 0.0));
    }
  }

  public void stop() {
    io.stop();
  }

  public void stopWheels() {
    io.stopWheels();
  }

  @AutoLogOutput
  public double getVelocityRad() {
    return inputs.velocityRadPerSec;
  }

  @AutoLogOutput
  public double getPositionRad() {
    return inputs.positionRad;
  }

  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }
}
