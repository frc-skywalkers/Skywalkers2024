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
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class FlywheelIOTalonFX implements FlywheelIO {
  private static final double GEAR_RATIO = 0.5;

  private final TalonFX leader = new TalonFX(22);
  private final TalonFX follower = new TalonFX(21);

  private final VelocityVoltage vel = new VelocityVoltage(0);

  private final MotionMagicVelocityVoltage mm_vel = new MotionMagicVelocityVoltage(0);

  private final StatusSignal<Double> leaderPosition = leader.getPosition();
  private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leader.getStatorCurrent();
  private final StatusSignal<Double> leaderError = leader.getClosedLoopError();
  private final StatusSignal<Double> leaderFF = leader.getClosedLoopFeedForward();
  private final StatusSignal<Double> leaderGoal = leader.getClosedLoopReference();

  private final StatusSignal<Double> followerPosition = follower.getPosition();
  private final StatusSignal<Double> followerVelocity = follower.getVelocity();
  private final StatusSignal<Double> followerAppliedVolts = follower.getMotorVoltage();
  private final StatusSignal<Double> followerCurrent = follower.getStatorCurrent();

  private double goalRadPerSec = 0.0;

  public FlywheelIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 45.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    MotionMagicConfigs mm_configs = config.MotionMagic;

    mm_configs.MotionMagicCruiseVelocity = 200.0;
    mm_configs.MotionMagicAcceleration = 600.0;
    mm_configs.MotionMagicJerk = 1200.0;

    Slot0Configs slot0 = config.Slot0;

    slot0.kP = 0.0009;
    slot0.kI = 0.0;
    slot0.kD = 0.0000025;
    slot0.kS = 0.466;
    slot0.kV = 0.018 * (2 * Math.PI) * GEAR_RATIO;
    slot0.kA = 0.01;

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    leader.setInverted(true);
    follower.setInverted(true);

    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0,
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        followerCurrent,
        leaderError,
        leaderFF,
        leaderGoal,
        followerPosition,
        followerVelocity);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        followerCurrent,
        leaderError,
        leaderFF,
        leaderGoal,
        followerPosition,
        followerVelocity);
    inputs.positionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(leaderVelocity.getValueAsDouble());
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()};

    inputs.followerPositionRad = Units.rotationsToRadians(followerPosition.getValueAsDouble());
    inputs.followerVelocityRadPerSec =
        Units.rotationsToRadians(followerVelocity.getValueAsDouble());
    inputs.followerAppliedVolts = followerAppliedVolts.getValueAsDouble();
    inputs.goalRadPerSec = goalRadPerSec;

    inputs.pidError = (leaderError.getValueAsDouble());
    inputs.leadGoal = Units.rotationsToRadians(leaderGoal.getValueAsDouble());
    inputs.feedforwardOut = leaderFF.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts));
    follower.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    // leader.setControl(
    //     new VelocityVoltage(
    //         Units.radiansToRotations(velocityRadPerSec), 0.0, false, 0, 0, false, false, false));
    // follower.setControl(
    //     new VelocityVoltage(
    //         Units.radiansToRotations(velocityRadPerSec * 0.8),
    //         0.0,
    //         false,
    //         0,
    //         0,
    //         false,
    //         false,
    //         false));

    leader.setControl(mm_vel.withVelocity(Units.radiansToRotations(velocityRadPerSec)).withSlot(0));
    follower.setControl(
        mm_vel.withVelocity(Units.radiansToRotations(velocityRadPerSec * 0.8)).withSlot(0));
    goalRadPerSec = velocityRadPerSec;
  }

  @Override
  public void stop() {
    leader.stopMotor();
    follower.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    // var config = new Slot0Configs();
    // config.kP = kP;
    // config.kI = kI;
    // config.kD = kD;
    // config.kS = 0.196;
    // config.kV = 0.018 * (2 * Math.PI);

    // leader.getConfigurator().apply(config);
    // follower.getConfigurator().apply(config);
  }
}
