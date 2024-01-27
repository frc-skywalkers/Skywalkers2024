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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;

public class PivotIOSim implements PivotIO {
  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1), 100.00, 0.05, 0.25, -Math.PI / 2, Math.PI / 2, true, 0.0);
  private ProfiledPIDController pid =
      new ProfiledPIDController(6.0, 0.0, 0.0, new TrapezoidProfile.Constraints(3, 2.5));

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;
  private double goalPos = 0.00;
  private double setpointPos = 0.00;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (closedLoop) {
      inputs.pidOut = pid.calculate(sim.getAngleRads());
      appliedVolts = MathUtil.clamp(inputs.pidOut + ffVolts, -10.0, 10.0);
      Logger.recordOutput("Pivot/PID", ffVolts);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);

    inputs.positionRad = sim.getAngleRads();
    inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.goalPos = goalPos;
    inputs.goalVel = pid.getSetpoint().velocity;
    inputs.setpointPos = pid.getSetpoint().position;
    inputs.ffVolts = ffVolts;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setPosition(double positionRad, double ffVolts) {
    closedLoop = true;
    // pid.setSetpoint(positionRad);
    // pid.setGoal(positionRad, 0.0);
    pid.setGoal(new State(positionRad, 0.0));
    this.ffVolts = ffVolts;
    goalPos = positionRad;
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
