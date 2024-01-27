package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class IntakeIOTalonFX implements IntakeIO {
  private static final double GEAR_RATIO = 1.5;

  private final TalonFX intake = new TalonFX(20);

  private final StatusSignal<Double> intakePosition = intake.getPosition();
  private final StatusSignal<Double> intakeVelocity = intake.getVelocity();
  private final StatusSignal<Double> intakeAppliedVolts = intake.getMotorVoltage();
  private final StatusSignal<Double> intakeCurrent = intake.getStatorCurrent();

  public IntakeIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intake.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, intakePosition, intakeVelocity, intakeAppliedVolts, intakeCurrent);
    intake.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(intakePosition, intakeVelocity, intakeAppliedVolts, intakeCurrent);
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(intakeVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.appliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.currentAmps = intakeCurrent.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    intake.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    intake.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec),
            0.0,
            false,
            ffVolts,
            0,
            false,
            false,
            false));
  }

  @Override
  public void stop() {
    intake.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    intake.getConfigurator().apply(config);
  }
}
