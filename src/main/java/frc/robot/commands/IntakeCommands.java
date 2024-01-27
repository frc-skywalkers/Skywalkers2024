package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;

public class IntakeCommands {
  private IntakeCommands() {}

  public static Command movetoPosition(Intake intake, double goal) {
    BooleanSupplier sup = () -> intake.atGoal;
    return Commands.runOnce(
            () -> {
              intake.goToPosition(goal); // pivot moving
            },
            intake)
        .andThen(Commands.waitUntil(sup));
  }
}
