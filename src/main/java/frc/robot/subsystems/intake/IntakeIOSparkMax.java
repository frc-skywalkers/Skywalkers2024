package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSparkMax implements IntakeIO {
  private static final double GEAR_RATIO = 40.0;
  private double goalPos = 0.00;
  private double goalVel = 0.0;

  private final CANSparkMax leader = new CANSparkMax(36, MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  // private final SparkRelativeEncoder encoder = new SparkRelativeEncoder(leader, )
  private final SparkPIDController pid = leader.getPIDController();

  private final ProfiledPIDController pidd =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));

  private final CANSparkMax wheel = new CANSparkMax(34, MotorType.kBrushless);
  private final RelativeEncoder wheelencoder = wheel.getEncoder();

  public IntakeIOSparkMax() {
    leader.restoreFactoryDefaults();
    wheel.restoreFactoryDefaults();

    leader.setIdleMode(IdleMode.kBrake);

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

    encoder.setPosition(0.0);
    pidd.reset(0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO) - 100;
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent()};
    inputs.goalPos = goalPos;
    inputs.goalVel = goalVel;

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
    // pid.setFF(ffVolts);
    goalPos = positionRad;

    double pa = ffVolts;
    double pb =
        pidd.calculate(Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO), goalPos);

    Logger.recordOutput("Intake/pa", pa);
    Logger.recordOutput("Intake/pb", pb);

    leader.setVoltage(pa + pb);
    goalVel = pidd.getSetpoint().velocity;
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    // pid.setP(kP);
    // pid.setI(kI);
    // pid.setD(kD);
    // pid.setFF(0);
    pidd.setPID(kP, kI, kD);
  }
}
