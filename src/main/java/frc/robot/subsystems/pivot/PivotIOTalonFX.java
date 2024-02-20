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
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

public class PivotIOTalonFX implements PivotIO {
  private static final double GEAR_RATIO = 200.00;

  private final TalonFX leader = new TalonFX(23);
  private final CANcoder cancoder = new CANcoder(16);

  private final StatusSignal<Double> leaderPosition = leader.getPosition();
  private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leader.getStatorCurrent();
  private final StatusSignal<Double> leaderOutput = leader.getClosedLoopOutput();
  private final StatusSignal<Double> leaderPIDError = leader.getClosedLoopError();
  private final StatusSignal<Double> leaderPIDRef = leader.getClosedLoopReference();

  private final StatusSignal<Double> absEncoderPos = cancoder.getAbsolutePosition();

  private final ProfiledPIDController pid =
      new ProfiledPIDController(10.0, 0.0, 0.10, new TrapezoidProfile.Constraints(0.3, 0.4));

  private final PositionVoltage m_voltagePosition =
      new PositionVoltage(0, 0, false, 0, 0, false, false, false);

  private final double absEncoderOffset = -0.024658;

  private double goalPos = 0.00;

  private double goalVel = 0.0;

  private final MotionMagicVoltage mm_volt = new MotionMagicVoltage(0);

  public PivotIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Slot0Configs pidConfigs = config.Slot0;

    pidConfigs.GravityType = GravityTypeValue.Arm_Cosine;
    pidConfigs.kS = 0.13;
    pidConfigs.kG = 0.16;
    pidConfigs.kP = 15.0;
    pidConfigs.kI = 0.0;
    pidConfigs.kD = 0.15;
    pidConfigs.kV = 24.0;
    pidConfigs.kA = 1.0;

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

    leader.setPosition(absEncoderPos.getValueAsDouble() - absEncoderOffset);
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
    inputs.absPos = Units.rotationsToRadians(absEncoderPos.getValueAsDouble() - absEncoderOffset);
    inputs.goalVel = goalVel;
    // Logger.recordOutput("Pivot/PID/Output", leaderOutput.getValueAsDouble());
    // Logger.recordOutput("Pivot/PID/Error", leaderPIDError.getValueAsDouble());
    // Logger.recordOutput("Pivot/PID/Ref", leaderPIDRef.getValueAsDouble());
  }

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
