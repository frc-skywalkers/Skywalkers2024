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

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.LightstripConstants;
import frc.robot.Constants.LightstripConstants.Ranges;
import frc.robot.subsystems.Lightstrip;

public class IndexerIOTalonFX implements IndexerIO {
  private static final double GEAR_RATIO = 1.5;

  private final TalonFX leader = new TalonFX(20);

  private Lightstrip lightstrip;

  private final StatusSignal<Double> leaderPosition = leader.getPosition();
  private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leader.getStatorCurrent();
  // private final StatusSignal<Double> leaderA = leader.getLimitSw

  private final DigitalInput index_sensor = new DigitalInput(9);
  private final DigitalInput shoot_sensor = new DigitalInput(8);

  // private final TimeOfFlight tofSensor = new TimeOfFlight(15);

  public IndexerIOTalonFX(Lightstrip ls) {
    lightstrip = ls;

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 45.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;

    leader.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    leader.optimizeBusUtilization();
    // tofSensor.setRangingMode(RangingMode.Short, 30);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    inputs.positionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {leaderCurrent.getValueAsDouble()};
    inputs.hasPiece = hasPiece();
    inputs.superHasPiece = superHasPiece();
    // inputs.tofDistance = tofSensor.getRange();
    // inputs.tofSD = tofSensor.getRangeSigma();

    if (inputs.hasPiece) {
      lightstrip.setColor(LightstripConstants.holdingState, Ranges.full);
    } else {
      lightstrip.setColor(LightstripConstants.defaultState, Ranges.full);
    }
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts));
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
  }

  @Override
  public void stop() {
    leader.stopMotor();
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    leader.getConfigurator().apply(config);
  }

  private boolean hasPiece() {
    return !index_sensor.get() || !shoot_sensor.get();
  }

  private boolean superHasPiece() {
    return !shoot_sensor.get();
  }
}
