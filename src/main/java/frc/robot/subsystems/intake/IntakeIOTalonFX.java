package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
  private static final double GEAR_RATIO = 40.0;
  private double goalPos = 0.00;
  private double goalVel = 0.0;

  private final TalonFX leader = new TalonFX(55);
  private final TalonFX wheel = new TalonFX(54);
  private final ProfiledPIDController pidd =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2 * Math.PI, 1.75 * Math.PI));

  private final StatusSignal<Double> leaderPosition = leader.getPosition();
  private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leader.getTorqueCurrent();
  // private final StatusSignal<Double> leaderGoal = leader.get

  private final StatusSignal<Double> wheelVelocity = wheel.getVelocity();
  private final StatusSignal<Double> wheelAppliedVolts = wheel.getMotorVoltage();
  private final StatusSignal<Double> wheelCurrent = wheel.getTorqueCurrent();

  private final MotionMagicVoltage mm_volt = new MotionMagicVoltage(0);

  private LinearFilter intakenPiece = LinearFilter.movingAverage(5);

  private final TimeOfFlight tofSensor = new TimeOfFlight(15);

  public IntakeIOTalonFX() {
    var leaderConfig = new TalonFXConfiguration();
    leaderConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var wheelConfig = new TalonFXConfiguration();
    wheelConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    wheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    Slot0Configs pidConfigs = leaderConfig.Slot0;

    pidConfigs.GravityType = GravityTypeValue.Arm_Cosine;
    pidConfigs.kS = 0.075;
    pidConfigs.kG = 0.2;
    pidConfigs.kP = 3.0;
    pidConfigs.kI = 0.0;
    pidConfigs.kD = 0.03;
    pidConfigs.kV = 4.80;
    pidConfigs.kA = 0.2;

    MotionMagicConfigs mm_configs = leaderConfig.MotionMagic;
    mm_configs.MotionMagicCruiseVelocity = IntakeConstants.mm_cruisevel;
    mm_configs.MotionMagicAcceleration = IntakeConstants.mm_accel;
    mm_configs.MotionMagicJerk = IntakeConstants.mm_jerk;

    FeedbackConfigs fdb_configs = leaderConfig.Feedback;

    fdb_configs.SensorToMechanismRatio = GEAR_RATIO;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = leader.getConfigurator().apply(leaderConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Real Error, Could not configure device. Error: " + status.toString());
    }

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = wheel.getConfigurator().apply(wheelConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Real Error, Could not configure device. Error: " + status.toString());
    }

    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    leader.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0, wheelVelocity, wheelAppliedVolts, wheelCurrent);
    wheel.optimizeBusUtilization();

    leader.setPosition(0.0);

    pidd.reset(0.0);
    tofSensor.setRangingMode(RangingMode.Long, 30);
    tofSensor.setRangeOfInterest(7, 7, 9, 9);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRad = leaderPosition.getValueAsDouble();
    inputs.velocityRadPerSec = leaderVelocity.getValueAsDouble();
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {leaderCurrent.getValueAsDouble(), wheelCurrent.getValueAsDouble()};
    inputs.goalPos = goalPos;
    inputs.goalVel = goalVel;

    inputs.wheelAppliedVolts = wheelAppliedVolts.getValueAsDouble();
    inputs.wheelVelocityRadPerSec = wheelVelocity.getValueAsDouble();
    inputs.tofDistance = tofSensor.getRange();
    inputs.tofSD = tofSensor.getRangeSigma();
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts));
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void runWheelVolts(double volts) {
    wheel.setControl(new VoltageOut(volts));
  }

  @Override
  public void stopWheels() {
    wheel.stopMotor();
  }

  @Override
  public void setPosition(double positionRad, double ffVolts) {
    leader.setControl(mm_volt.withPosition(Units.radiansToRotations(positionRad)).withSlot(0));
  }

  public double getCurrent() {
    // return intakenPiece.calculate(wheel.getOutputCurrent());
    return wheelCurrent.getValueAsDouble();
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
