package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

public class PivotIOTalonFX implements PivotIO {
  private final TalonFX pivot = new TalonFX(21);
  private static final double GEAR_RATIO = 45.0 * 44.0 / 18.0;

  private final StatusSignal<Double> Position = pivot.getPosition(); // phoenix 6 stuff
  private final StatusSignal<Double> Velocity = pivot.getVelocity(); // rotations per second
  private final StatusSignal<Double> AppliedVolts = pivot.getMotorVoltage();
  private final StatusSignal<Double> Current = pivot.getStatorCurrent();

  public boolean isZeroed = false;

  public PivotIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.00;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivot.getConfigurator().apply(config);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, Position, Velocity, AppliedVolts, Current); // things to update
    pivot.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(Position, Velocity, AppliedVolts, Current);
    inputs.positionRad = Units.rotationsToRadians(Position.getValueAsDouble()) / GEAR_RATIO;
    inputs.velocityRadPerSec = Units.rotationsToRadians(Velocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.appliedVolts = AppliedVolts.getValueAsDouble();
    inputs.currentAmps = Current.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    volts = MathUtil.clamp(volts, -8, 8);
    pivot.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    pivot.set(velocityRadPerSec);
  }

  @Override
  public void stop() {
    pivot.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    pivot.getConfigurator().apply(config);
  }

  public void reset() {
    pivot.setPosition(0);
  }
}
