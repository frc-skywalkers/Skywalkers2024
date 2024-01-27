package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class PivotIONeo implements PivotIO {
  private static final double GEAR_RATIO = 44.0 * 45.0 / 18.0;

  private final CANSparkMax pivot = new CANSparkMax(20, MotorType.kBrushless);
  private final RelativeEncoder encoder = pivot.getEncoder();
  private final SparkPIDController pid = pivot.getPIDController();

  public PivotIONeo() {
    pivot.restoreFactoryDefaults();

    pivot.setCANTimeout(250);

    pivot.setInverted(false);

    pivot.enableVoltageCompensation(12.0);
    pivot.setSmartCurrentLimit(30);

    pivot.burnFlash();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = pivot.getAppliedOutput() * pivot.getBusVoltage();
    inputs.currentAmps = pivot.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    pivot.setVoltage(volts);
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
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }

  public void reset() {
    encoder.setPosition(0);
  }
}
