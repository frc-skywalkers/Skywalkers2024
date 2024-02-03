// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisualizerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import org.littletonrobotics.junction.Logger;

public class Visualizer extends SubsystemBase {
  /** Creates a new Visualizer. */
  Intake intake;

  Pivot pivot;
  Mechanism2d mech;

  MechanismRoot2d intakeRoot;
  MechanismRoot2d pivotRoot;

  private MechanismLigament2d intakeMount;
  private MechanismLigament2d intakeHandle;
  private MechanismLigament2d intakeDropDown;

  private MechanismLigament2d pivotMount;
  private MechanismLigament2d shooterHandle;
  private MechanismLigament2d shooter;

  // private Mechanism

  public Visualizer(Intake intake, Pivot pivot) {
    this.intake = intake;
    this.pivot = pivot;
    mech = new Mechanism2d(29.0, 29.0);

    intakeRoot = mech.getRoot("intake", 5, 3);
    pivotRoot = mech.getRoot("pivot", 25, 3);

    intakeMount =
        intakeRoot.append(
            new MechanismLigament2d("Intake Mount", 2, VisualizerConstants.intakeMountAngle));
    intakeHandle =
        intakeMount.append(
            new MechanismLigament2d("Intake Handle", 6.5, -VisualizerConstants.intakeMountAngle));
    intakeDropDown = intakeHandle.append(new MechanismLigament2d("Drop Down Intake", 5.5, 45));

    pivotMount =
        pivotRoot.append(
            new MechanismLigament2d("Pivot Mount", 20, VisualizerConstants.pivotMountAngle));
    shooterHandle =
        pivotMount.append(
            new MechanismLigament2d(
                "Shooter Handle", 4, 0 - VisualizerConstants.pivotMountAngle + 180));
    shooter = shooterHandle.append(new MechanismLigament2d("Shooter", 16, 45));
    Logger.recordOutput("SuperStructure/mechanism", mech);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeHandle.setAngle(
        180 - intake.getPositionRad() * (180.00 / Math.PI) - VisualizerConstants.intakeMountAngle);
    shooterHandle.setAngle(
        180 + pivot.getPositionRad() * (180.000 / Math.PI) - VisualizerConstants.pivotMountAngle);

    Logger.recordOutput("SuperStructure/mechanism", mech);
  }
}
