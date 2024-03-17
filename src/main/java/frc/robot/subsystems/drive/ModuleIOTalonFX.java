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

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final StatusSignal<Double> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;

  // Gear ratios for SDS MK4i L2, adjust as necessary
  private final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  private final MotionMagicVelocityVoltage drive_mm = new MotionMagicVelocityVoltage(0.0);
  private final MotionMagicVoltage turn_mm = new MotionMagicVoltage(0.0);

  public ModuleIOTalonFX(int index) {
    switch (index) {
      case 0:
        driveTalon = new TalonFX(1, "CANivore");
        turnTalon = new TalonFX(2, "CANivore");
        cancoder = new CANcoder(3, "CANivore");
        absoluteEncoderOffset =
            new Rotation2d(0.065674 * 2 * Math.PI + Math.PI); // MUST BE CALIBRATED
        break;
      case 1:
        driveTalon = new TalonFX(4, "CANivore");
        turnTalon = new TalonFX(5, "CANivore");
        cancoder = new CANcoder(6, "CANivore");
        absoluteEncoderOffset = new Rotation2d((-0.354736 * 2 * Math.PI)); // MUST BE CALIBRATED
        break;
      case 2:
        driveTalon = new TalonFX(10, "CANivore");
        turnTalon = new TalonFX(11, "CANivore");
        cancoder = new CANcoder(12, "CANivore");
        absoluteEncoderOffset =
            new Rotation2d(0.223145 * 2 * Math.PI + Math.PI); // MUST BE CALIBRATED
        break;
      case 3:
        driveTalon = new TalonFX(7, "CANivore");
        turnTalon = new TalonFX(8, "CANivore");
        cancoder = new CANcoder(9, "CANivore");
        absoluteEncoderOffset = new Rotation2d(-0.0632 * 2 * Math.PI); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 50.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
    driveConfig.Slot0.kP = 0.25;
    driveConfig.Slot0.kD = 0.01;
    driveConfig.Slot0.kS = 0.148;
    driveConfig.Slot0.kV = 0.12 * DRIVE_GEAR_RATIO;
    driveConfig.Slot0.kA = 0.1;

    driveConfig.MotionMagic.MotionMagicCruiseVelocity = 15.0;
    driveConfig.MotionMagic.MotionMagicAcceleration = 10.0;
    driveConfig.MotionMagic.MotionMagicJerk = 12.5;

    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    // drive_mm.withUpdateFreqHz(index)

    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimit = 35.0;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.Feedback.SensorToMechanismRatio = TURN_GEAR_RATIO;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.Slot0.kP = 3.5;
    turnConfig.Slot0.kD = 0.05;
    turnConfig.Slot0.kS = 0.2;
    turnConfig.Slot0.kV = 0.12 * TURN_GEAR_RATIO;
    turnConfig.Slot0.kA = 0.05;

    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 1.5;
    turnConfig.MotionMagic.MotionMagicAcceleration = 4.5;
    turnConfig.MotionMagic.MotionMagicJerk = 4.5;
    turnTalon.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(true);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};

    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveMMV(double velocityRadPerSec) {
    driveTalon.setControl(
        drive_mm.withVelocity(Units.radiansToRotations(velocityRadPerSec)).withSlot(0));
  }

  @Override
  public void setTurnMM(double rad) {
    turnTalon.setControl(turn_mm.withPosition(Units.radiansToRotations(rad)).withSlot(0));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }
}
