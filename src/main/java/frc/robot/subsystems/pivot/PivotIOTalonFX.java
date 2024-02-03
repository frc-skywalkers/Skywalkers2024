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
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.PivotConstants;

public class PivotIOTalonFX implements PivotIO {
  private static final double GEAR_RATIO = 60.00;

  private final TalonFX leader = new TalonFX(0);
  private final CANcoder cancoder = new CANcoder(1);

  private final StatusSignal<Double> leaderPosition = leader.getPosition();
  private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leader.getStatorCurrent();
  private double goalPos = 0.00;

  public PivotIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leader.getConfigurator().apply(config);

    MotionMagicConfigs mm_configs = config.MotionMagic;

    mm_configs.MotionMagicCruiseVelocity = PivotConstants.mm_cruisevel;
    mm_configs.MotionMagicAcceleration = PivotConstants.mm_accel;
    mm_configs.MotionMagicJerk = PivotConstants.mm_jerk;

    FeedbackConfigs f_configs = config.Feedback;

    f_configs.FeedbackRemoteSensorID = cancoder.getDeviceID();
    f_configs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    f_configs.RotorToSensorRatio = GEAR_RATIO;

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    leader.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    inputs.positionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(leaderVelocity.getValueAsDouble());
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {leaderCurrent.getValueAsDouble()};
    inputs.goalPos = goalPos;
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts));
  }

  @Override
  public void setPosition(double positionRad, double ffVolts) {
    leader.setControl(
        new MotionMagicVoltage(
            Units.radiansToRotations(positionRad), false, ffVolts, 0, false, false, false));
    goalPos = positionRad;
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePIDFF(double kP, double kI, double kD, double kG, double kS) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    leader.getConfigurator().apply(config);
    // leader.Slot0Configs = config;
  }
}
