// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.PivotConstants;
import org.littletonrobotics.junction.Logger;

public class PivotIOTalonFX implements PivotIO {
  private static final double GEAR_RATIO = 125.00 * 34.00 / 22.0;

  private final TalonFX leader = new TalonFX(61);
  private final TalonFX follower = new TalonFX(62);
  private final CANcoder cancoder = new CANcoder(16);

  private final StatusSignal<Double> leaderPosition = leader.getPosition();
  private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leader.getStatorCurrent();
  private final StatusSignal<Double> leaderOutput = leader.getClosedLoopOutput();
  private final StatusSignal<Double> leaderPIDError = leader.getClosedLoopError();
  private final StatusSignal<Double> leaderPIDRef = leader.getClosedLoopReference();

  private final StatusSignal<Double> followerPosition = follower.getPosition();
  private final StatusSignal<Double> followerVelocity = follower.getVelocity();
  private final StatusSignal<Double> followerAppliedVolts = follower.getMotorVoltage();
  private final StatusSignal<Double> followerCurrent = follower.getStatorCurrent();

  private final StatusSignal<Double> absEncoderPos = cancoder.getAbsolutePosition();

  private final ProfiledPIDController pid =
      new ProfiledPIDController(10.0, 0.0, 0.10, new TrapezoidProfile.Constraints(0.3, 0.4));

  private final PositionVoltage m_voltagePosition =
      new PositionVoltage(0, 0, false, 0, 0, false, false, false);

  private final double absEncoderOffset = 0.60831144;

  private double goalPos = 0.00;

  private double goalVel = 0.0;

  private final MotionMagicVoltage mm_volt = new MotionMagicVoltage(0);

  private double outputOffset = 0.0;
  private double initialAbs = 0.0;

  public PivotIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 45.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Slot0Configs pidConfigs = config.Slot0;

    pidConfigs.GravityType = GravityTypeValue.Arm_Cosine;
    pidConfigs.kS = 0.1;
    pidConfigs.kG = 0.14; // 0.2
    pidConfigs.kP = 10.0;
    pidConfigs.kI = 0.0;
    pidConfigs.kD = 0.20;
    pidConfigs.kV = 0.12 * GEAR_RATIO;
    pidConfigs.kA = 0.25;

    MotionMagicConfigs mm_configs = config.MotionMagic;

    mm_configs.MotionMagicCruiseVelocity = PivotConstants.mm_cruisevel;
    mm_configs.MotionMagicAcceleration = PivotConstants.mm_accel;
    mm_configs.MotionMagicJerk = PivotConstants.mm_jerk;

    FeedbackConfigs fdb_configs = config.Feedback;

    fdb_configs.SensorToMechanismRatio = GEAR_RATIO;

    // leader.getConfigurator().apply(config);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = leader.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Real Error, Could not configure device. Error: " + status.toString());
    }

    for (int i = 0; i < 5; ++i) {
      status = follower.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Real Error, Could not configure device. Error: " + status.toString());
    }

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    // follower.setInverted(true);
    follower.setControl(new Follower(leader.getDeviceID(), true));

    // FeedbackConfigs f_configs = config.Feedback;

    // f_configs.FeedbackRemoteSensorID = cancoder.getDeviceID();
    // f_configs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // f_configs.RotorToSensorRatio = GEAR_RATIO;

    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0,
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        absEncoderPos,
        leaderOutput,
        leaderPIDError,
        leaderPIDRef);
    leader.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0, followerPosition, followerVelocity, followerAppliedVolts, followerCurrent);

    leader.setPosition(absEncoderPos.getValueAsDouble() - absEncoderOffset);
    follower.setPosition(absEncoderPos.getValueAsDouble() - absEncoderOffset);

    initialAbs = absEncoderPos.getValueAsDouble();
    outputOffset = initialAbs - absEncoderOffset;
    pid.reset(absEncoderPos.getValueAsDouble() - absEncoderOffset);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        absEncoderPos,
        leaderOutput,
        leaderPIDError,
        leaderPIDRef);
    inputs.positionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(leaderVelocity.getValueAsDouble());
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {leaderCurrent.getValueAsDouble()};
    inputs.goalPos = goalPos;
    inputs.absPos = Units.rotationsToRadians(absEncoderPos.getValueAsDouble());
    inputs.goalVel = goalVel;
    Logger.recordOutput("Pivot/outputOffset", outputOffset);
    Logger.recordOutput("Pivot/initialAbs", initialAbs);
    // Logger.recordOutput("Pivot/PID/Output", leaderOutput.getValueAsDouble());
    // Logger.recordOutput("Pivot/PID/Error", leaderPIDError.getValueAsDouble());
    // Logger.recordOutput("Pivot/PID/Ref", leaderPIDRef.getValueAsDouble());
  }

  // private double getAbsolutePosition() {
  // return abs
  // }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts));
  }

  @Override
  public void setPosition(double positionRad, double ffVolts) {
    // leader.setControl(
    // new PositionVoltage(
    // Units.radiansToRotations(positionRad), 0.0, false, ffVolts, 0, false, false, false));
    double setVolts =
        pid.calculate(leaderPosition.getValueAsDouble(), Units.radiansToRotations(positionRad));

    // leader.setControl(new VoltageOut(setVolts + ffVolts));
    goalVel = pid.getSetpoint().velocity;
    // goalPos = pid.getSetpoint().position;
    goalPos = positionRad;

    leader.setControl(mm_volt.withPosition(Units.radiansToRotations(positionRad)).withSlot(0));

    // leader.setControl(m_voltagePosition.withPosition(Units.radiansToRotations(positionRad)));
    // leader.setControl(
    // new PositionVoltage(Units.radiansToRotations(positionRad), 0.0, false, ffVolts, 0, false,
    // false, false));

  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePIDFF(double kP, double kI, double kD, double kG, double kS) {
    // var config = new Slot0Configs();
    // config.kP = kP * 50;

    // config.kI = kI;
    // config.kD = kD;

    // config.kV = 24.0;
    // config.kA = 1.0;
    // leader.getConfigurator().apply(config);
    // leader.Slot0Configs = config;
  }
}
