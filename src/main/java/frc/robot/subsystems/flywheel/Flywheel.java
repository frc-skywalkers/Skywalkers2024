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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  private double desiredRPM = 0.000;

  private double autoShootRPM = 0.000;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new SimpleMotorFeedforward(0.196, 0.0268); // need to determine
        io.configurePID(0.0009, 0.0, 0.000025); // need to determine
        break;
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.02);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
    updateTable();
  }

  public void updateTable() {
    // ShooterConstants.RPMAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 0.0, 5000.0);
    // ShooterConstants.RPMAngleMap.put(1.414, 5000.0);
    // ShooterConstants.RPMAngleMap.put(1.892, 5500.0);
    // ShooterConstants.RPMAngleMap.put(2.225, 5750.0);
    // ShooterConstants.RPMAngleMap.put(2.071, 5750.0);
    // ShooterConstants.RPMAngleMap.put(2.416, 6000.0);
    // ShooterConstants.RPMAngleMap.put(2.6521, 6750.0);

    ShooterConstants.RPMAngleMap.put(2.405, 5500.0);
    ShooterConstants.RPMAngleMap.put(2.128, 5500.0);
    ShooterConstants.RPMAngleMap.put(1.950, 5250.0);
    ShooterConstants.RPMAngleMap.put(1.448, 5000.0);
    ShooterConstants.RPMAngleMap.put(2.807, 6000.0);
    ShooterConstants.RPMAngleMap.put(3.251, 6500.0);
    ShooterConstants.RPMAngleMap.put(3.658, 6800.0);

    // ShooterConstants.RPMAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 25.0,
    // 5000.0);
    // ShooterConstants.RPMAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 13.5,
    // 5000.0);
    // ShooterConstants.RPMAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 34.0,
    // 5550.0);
    // ShooterConstants.RPMAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 43.0,
    // 5550.0);
    // ShooterConstants.RPMAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 50.0,
    // 6000.0);
    // ShooterConstants.RPMAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 84.75,
    // 6750.0);
    // ShooterConstants.RPMAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 68.0,
    // 7250.0);
    // ShooterConstants.RPMAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 101.0,
    // 8250.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));
    desiredRPM = velocityRPM;

    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    // io.stop();
    io.setVoltage(-0.75);
  }

  public boolean atDesiredRPM() {
    return Math.abs(getVelocityRPM() - desiredRPM) < ShooterConstants.tolerance;
  }

  public boolean atDesiredRPM(double requestRPM) {
    boolean ret = (Math.abs(getVelocityRPM() - requestRPM) < ShooterConstants.tolerance);
    Logger.recordOutput("Flywheel/atSetpointRPM", ret);
    return ret;
  }

  public void updateAutoShoot(double ang) {
    this.autoShootRPM = ang;
  }

  public boolean shootRevved() {
    return atDesiredRPM(this.autoShootRPM);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.followerVelocityRadPerSec;
  }

  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }
}
