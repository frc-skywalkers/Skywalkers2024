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

package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class FlywheelIOTalonFX implements FlywheelIO {
  private static final double GEAR_RATIO = 1.5;

  private final TalonFX leader = new TalonFX(20);
  private final TalonFX follower = new TalonFX(21);

  private final StatusSignal<Double> leaderPosition = leader.getPosition();
  private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leader.getStatorCurrent();

  private final StatusSignal<Double> followerPosition = follower.getPosition();
  private final StatusSignal<Double> followerVelocity = follower.getVelocity();
  private final StatusSignal<Double> followerAppliedVolts = follower.getMotorVoltage();
  private final StatusSignal<Double> followerCurrent = follower.getStatorCurrent();

  public FlywheelIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);

    follower.setInverted(true);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        followerPosition,
        followerVelocity,
        followerAppliedVolts,
        followerCurrent);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    inputs.positionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()};

    inputs.followerPositionRad =
        Units.rotationsToRadians(followerPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.followerVelocityRadPerSec =
        Units.rotationsToRadians(followerVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.followerAppliedVolts = followerAppliedVolts.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts));
    follower.setControl(new VoltageOut(volts * 0.8));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    leader.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec),
            0.0,
            false,
            ffVolts,
            0,
            false,
            false,
            false));
    follower.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec * 0.8),
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
    leader.stopMotor();
    follower.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
  }
}
