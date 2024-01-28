package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;

public class IntakeIOSparkMax implements IntakeIO {
  private static final double GEAR_RATIO = 60.0;
  private double goalPos = 0.00;

  private final CANSparkMax leader = new CANSparkMax(0, MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final SparkPIDController pid = leader.getPIDController();

  private final CANSparkMax wheel = new CANSparkMax(1, MotorType.kBrushless);
  private final RelativeEncoder wheelencoder = wheel.getEncoder();

  public IntakeIOSparkMax() {
    leader.restoreFactoryDefaults();
    wheel.restoreFactoryDefaults();

    leader.setCANTimeout(250);
    wheel.setCANTimeout(250);

    leader.setInverted(false);
    wheel.setInverted(false);

    leader.enableVoltageCompensation(12.0);
    wheel.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(30);
    wheel.setSmartCurrentLimit(30);

    leader.burnFlash();
    wheel.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent()};
    inputs.goalPos = goalPos;

    inputs.wheelAppliedVolts = wheel.getAppliedOutput() * wheel.getBusVoltage();
    inputs.wheelPositionRad = Units.rotationsToRadians(wheelencoder.getPosition() / 1.5); //
    inputs.wheelVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / 1.5);
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void runWheelVolts(double volts) {
    wheel.setVoltage(volts);
  }

  @Override
  public void stopWheels() {
    wheel.stopMotor();
  }

  @Override
  public void setPosition(double positionRad, double ffVolts) {
    pid.setFF(ffVolts);
    goalPos = positionRad;
    pid.setReference(positionRad, ControlType.kPosition);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
    pid.setFF(0);
  }
}
