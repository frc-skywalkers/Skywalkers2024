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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FlywheelCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.Lightstrip;
import frc.robot.subsystems.Visualizer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOTalonFX;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Flywheel flywheel;
  private final Pivot pivot;
  private final Intake intake;
  private final Indexer indexer;
  private final Visualizer visualizer;
  // private final Vision vision;
  public final Lightstrip lightstrip;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  // private final CommandPS4Controller controller = new CommandPS4Controller(0);

  private final CommandXboxController operator = new CommandXboxController(1);
  //   private final CommandPS4Controller operator = new CommandPS4Controller(1);

  //   private final CommandJoystick op = new CommandJoystick(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    lightstrip = new Lightstrip();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // drive =
        //     new Drive(
        //         new GyroIOPigeon2(false),
        //         new ModuleIOSparkMax(0),
        //         new ModuleIOSparkMax(1),
        //         new ModuleIOSparkMax(2),
        //         new ModuleIOSparkMax(3));
        // flywheel = new Flywheel(new FlywheelIOSparkMax());
        // drive =
        //     new Drive(
        //         new GyroIOPigeon2(true),
        //         new ModuleIOTalonFX(0),
        //         new ModuleIOTalonFX(1),
        //         new ModuleIOTalonFX(2),
        //         new ModuleIOTalonFX(3));
        // flywheel = new Flywheel(new FlywheelIOSim());
        // pivot = new Pivot(new PivotIOSim());
        // intake = new Intake(new IntakeIOSim());
        // indexer = new Indexer(new IndexerIOSim());
        // visualizer = new Visualizer(intake, pivot);

        drive =
            new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        flywheel = new Flywheel(new FlywheelIOTalonFX());
        pivot = new Pivot(new PivotIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        visualizer = new Visualizer(intake, pivot);

        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new ModuleIOSim(),
        //         new ModuleIOSim(),
        //         new ModuleIOSim(),
        //         new ModuleIOSim());
        // flywheel = new Flywheel(new FlywheelIOTalonFX());
        // pivot = new Pivot(new PivotIOTalonFX());
        // intake = new Intake(new IntakeIOSparkMax());
        // indexer = new Indexer(new IndexerIOTalonFX());
        // visualizer = new Visualizer(intake, pivot);

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        flywheel = new Flywheel(new FlywheelIOSim());
        pivot = new Pivot(new PivotIOSim());
        intake = new Intake(new IntakeIOSim());
        indexer = new Indexer(new IndexerIOSim());
        visualizer = new Visualizer(intake, pivot);

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        flywheel = new Flywheel(new FlywheelIO() {});
        pivot = new Pivot(new PivotIOSim());
        intake = new Intake(new IntakeIOSim());
        indexer = new Indexer(new IndexerIOSim());
        visualizer = new Visualizer(intake, pivot);

        break;
    }

    SmartDashboard.putNumber("Indexer Volt Wanted", 0.0);
    SmartDashboard.putNumber("Indexer Volt Wanted 1", 0.0);

    SmartDashboard.putNumber("Shooter Volt Wanted", 0.0);
    SmartDashboard.putNumber("Pivot Angle Wanted", -1.0);
    SmartDashboard.putNumber("Pivot Angle Wanted 1", 0.8);

    SmartDashboard.putNumber("Shooter RPM Wanted", 3000);
    SmartDashboard.putNumber("Indexer Position Back", 1.0);

    Logger.recordOutput("Intake/intakingPiece", false);

    Logger.recordOutput("Indexer/outtaking", false);
    Logger.recordOutput("Pivot/aiming", false);
    Logger.recordOutput("Pivot/I'm super evil", false);

    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
                () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
            .withTimeout(5.0));

    // NamedCommands.registerCommand("aim + rev flywheel", FlywheelCommands.autoShoot(flywheel,
    // drive, pivot));

    NamedCommands.registerCommand(
        "intake handoff",
        IntakeCommands.intakeHandoff(intake, indexer, pivot, lightstrip)
            .andThen(IntakeCommands.indexSequence(intake, indexer, pivot)));

    NamedCommands.registerCommand(
        "Pivot Rev", FlywheelCommands.prepSubwoofer(flywheel, pivot, indexer));

    NamedCommands.registerCommand(
        "Shoot", FlywheelCommands.shoot(pivot, flywheel, indexer, intake));

    NamedCommands.registerCommand(
        "Shoot Sequence",
        Commands.race(
            FlywheelCommands.prepSubwoofer(flywheel, pivot, indexer),
            FlywheelCommands.shoot(pivot, flywheel, indexer, intake)));

    Logger.recordOutput("Alliance", DriverStation.getAlliance().isPresent());

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    autoChooser.addOption(
        "Flywheel FF Characterization",
        new FeedForwardCharacterization(
            flywheel, flywheel::runVolts, flywheel::getCharacterizationVelocity));
    autoChooser.addOption(
        "Arm FF Characterization",
        new FeedForwardCharacterization(
            pivot, pivot::runVolts, pivot::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    if (!Constants.isRed) {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -controller.getLeftY(),
              () -> -controller.getLeftX(),
              () -> -controller.getRightX(),
              () -> controller.getLeftTriggerAxis(),
              () -> controller.getRightTriggerAxis()));
    } else {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> controller.getLeftY(),
              () -> controller.getLeftX(),
              () -> -controller.getRightX(), // rotation not flipped
              () -> controller.getLeftTriggerAxis(),
              () -> controller.getRightTriggerAxis()));
    }

    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // flywheel.setDefaultCommand(FlywheelCommands.autoShoot(flywheel, drive));
    // controller.x().onTrue(IntakeCommands.spit(intake));

    controller.leftBumper().onTrue(DriveCommands.autoAlignAmp(drive, controller));

    // intake.setDefaultCommand(
    //     Commands.runOnce(
    //         () -> {
    //           intake.setPosition(IntakeConstants.handoff);
    //           intake.stopWheels();
    //         },
    //         intake));

    // pivot.setDefaultCommand(FlywheelCommands.autoPivotAim(pivot, drive));
    indexer.setDefaultCommand(
        Commands.runOnce(() -> indexer.runVolts(IndexerConstants.holdVolts), indexer));
    // controller.a().onTrue(Commands.run(() -> pivot.setPosition(-0.25), pivot));
    // controller.b().onTrue(Commands.run(() -> pivot.setPosition(0.0), pivot));

    indexer.setDefaultCommand(Commands.runOnce(() -> indexer.runVolts(0), indexer));

    // flywheel.setDefaultCommand(
    //     FlywheelCommands.shootWhileMove(
    //         flywheel, drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // controller.y().onTrue(Commands.runOnce(() -> drive.ampAligned = true));

    // operator.leftTrigger().onTrue(Commands.runOnce(() -> drive.setShootMode(true), drive));
    // operator.leftTrigger().onFalse(Commands.runOnce(() -> drive.setShootMode(false), drive));

    // controller
    //     .a()
    //     .whileTrue(
    //         Commands.runOnce(
    //             () -> flywheel.runVelocity(SmartDashboard.getNumber("Shooter Volt Wanted",
    // 4500))));

    // pivot.setDefaultCommand(Commands.run(() -> pivot.runVolts(controller.getLeftX() * 8.0),
    // pivot));

    // controller
    //     .a()
    //     .whileTrue(
    //         Commands.run(
    //             () -> pivot.runVolts(SmartDashboard.getNumber("Shooter Volt Wanted", 0)),
    // pivot));

    // controller
    // .a()
    // .whileTrue(Commands.runOnce(() -> flywheel.runVelocity(controller.getLeftX() * 3000)));

    // flywheel.setDefaultCommand(
    // Commands.runOnce(() -> flywheel.runVelocity(controller.getLeftX() * 3000), flywheel));

    // controller
    //     .b()
    //     .whileTrue(
    //         Commands.runOnce(
    //             () -> indexer.runVolts(SmartDashboard.getNumber("Indexer Volt Wanted", 0))));

    // controller.x().onTrue(Commands.runOnce(() -> intake.runVolts(1.0)));

    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> intake.runVolts(SmartDashboard.getNumber("Shooter Volt Wanted", 0.0))));

    // controller
    //     .x()
    //     .onTrue(Commands.run(() -> intake.setPosition(0.05)).until(() ->
    // intake.atPosition(0.05)));

    // controller.a().onTrue(Commands.runOnce(() -> intake.runVolts(-1.0)));

    // controller
    //     .y()
    //     .onTrue(Commands.run(() -> intake.setPosition(3.0)).until(() -> intake.atPosition(3.0)));

    // controller.leftBumper().onTrue(Commands.run(() -> intake.reset))
    // controller.a().onTrue(Commands.runOnce(() -> intake.runWheelVolts(-6.0)));
    // controller.b().onTrue(Commands.runOnce(() -> intake.runWheelVolts(0.0)));

    // controller.axisGreaterThan(2, 0.5).onTrue(new IntakePiece(intake, pivot, indexer));
    // controller
    //     .axisGreaterThan(0, 0.5)
    //     .onTrue(Commands.runOnce(IntakePiece(intake, pivot, indexer)));

    // operator.leftBumper().onTrue(new IntakePiece(intake, pivot, indexer).andThen(new
    // IndexPiece(intake, indexer, pivot)));

    // controller.a().onTrue(Commands.runOnce(() -> intake.setPosition(1.5)));
    // controller.pov(0).onTrue(Commands.runOnce(() -> intake.setPosition(0)));

    // press button 1 and heres what happens
    // 1. intake drops down + pick up piece
    // 2. handoff position for intake + pivot
    // 3. outtake through intake + index

    // controller
    //     .x()
    //     .onTrue(
    //         IntakeCommands.intakePiece(intake)
    //             .andThen(IntakeCommands.passPieceIntake(intake, pivot, indexer))
    //             .andThen(IntakeCommands.transferPiece(intake, indexer)));

    // operator
    //     .b()
    //     .onTrue(
    //         IntakeCommands.intakeHandoff(intake, indexer, pivot, lightstrip)
    //             .unless(() -> indexer.hasPiece()));

    // operator
    //     .povDown()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               lightstrip.tempColor(
    //                   LightstripConstants.successSignal, LightstripConstants.Ranges.full);
    //             },
    //             lightstrip));

    operator.povDown().onTrue(IntakeCommands.spit(intake));

    operator
        .povLeft()
        .onTrue(
            Commands.run(
                    () -> {
                      pivot.runVolts(operator.getLeftX() * 8.0);
                    },
                    pivot)
                .until(() -> operator.povRight().getAsBoolean()));

    operator
        .povUp()
        .onTrue(
            IntakeCommands.intakeHandoff(intake, indexer, pivot, lightstrip)
                .andThen(IntakeCommands.indexSequence(intake, indexer, pivot)));

    operator
        .rightBumper()
        .onTrue(
            Commands.run(
                () -> {
                  intake.setPosition(IntakeConstants.handoff);
                  intake.stopWheels();
                },
                intake));

    // operator.x().onTrue(IntakeCommands.homeIntake(intake));

    // operator.x().onTrue(IntakeCommands.spit(intake));

    // operator.a().onTrue(IntakeCommands.indexSequence(intake, indexer, pivot));

    // operator.a().onTrue(Commands.run(() -> intake.setPosition(IntakeConstants.handoff), intake));

    // operator.b().onTrue(Commands.run(() -> intake.runWheelVolts(4.0), intake));

    // operator
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> flywheel.runVelocity(SmartDashboard.getNumber("Shooter RPM Wanted",
    // 3500))));

    pivot.setDefaultCommand(
        Commands.run(
            () -> {
              // pivot.runVolts(0.0);
              pivot.setPosition(-1.5);
              //   pivot.runVolts(operator.getLeftX() * 6.0);
              Logger.recordOutput("Pivot/I'm super evil", true);
            },
            pivot));

    flywheel.setDefaultCommand(Commands.run(() -> flywheel.runVelocity(0.0), flywheel));

    operator.leftBumper().whileTrue(FlywheelCommands.shoot(pivot, flywheel, indexer, intake));
    // operator.rightBumper().whileTrue(FlywheelCommands.outtake(indexer, intake));

    // operator.x().onTrue()

    // operator
    //     .x()
    //     .whileTrue(
    //         Commands.run(
    //             () -> {
    //               intake.runWheelVolts(10.0);
    //             },
    //             intake));

    // operator
    //     .povDown()
    //     .whileTrue(
    //         Commands.run(
    //             () -> {
    //               pivot.runVolts(4.0);
    //             },
    //             intake));
    // operator
    //     .povUp()
    //     .whileTrue(
    //         Commands.run(
    //             () -> {
    //               pivot.runVolts(-4.0);
    //             },
    //             intake));

    // operator
    //     .povLeft()
    //     .whileTrue(
    //         Commands.run(
    //             () -> {
    //               intake.runWheelVolts(IntakeConstants.intakeVolts);
    //             },
    //             intake));
    // intake.setDefaultCommand(Commands.run() -> {

    // });
    // controller`
    //     .x()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> flywheel.runVelocity(SmartDashboard.getNumber("Shooter RPM Wanted", 3500)),
    //             flywheel));
    // operator.y().onTrue(Commands.runOnce(() -> flywheel.runVelocity(0.0)));

    // operator
    //     .leftBumper()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> pivot.setPosition(SmartDashboard.getNumber("Pivot Angle Wanted", 0.45))));

    // operator
    //     .leftTrigger()
    //     .whileTrue(
    //         Commands.run(
    //             () -> {
    //               pivot.setPosition(SmartDashboard.getNumber("Pivot Angle Wanted", 0.0));
    //               flywheel.runVelocity(SmartDashboard.getNumber("Shooter RPM Wanted", 0.0));
    //               //   pivot.setPosition(SmartDashboard.getNumber("Pivot Angle Wanted", -1.0));
    //               //   flywheel.runVelocity(SmartDashboard.getNumber("Shooter RPM Wanted", 0.0));
    //             },
    //             pivot,
    //             flywheel));

    // operator
    //     .leftTrigger()
    //     .whileTrue(
    //         FlywheelCommands.autoPivotAim(pivot, drive)
    //             .alongWith(FlywheelCommands.autoShoot(flywheel, drive)));

    operator
        .leftTrigger()
        .whileTrue(
            Commands.run(
                () -> {
                  pivot.setPosition(-1.05);
                },
                pivot));

    operator
        .rightTrigger()
        .whileTrue(
            Commands.run(
                () -> {
                  flywheel.runVelocity(4500);
                },
                flywheel));

    /*operator
        .rightTrigger()
        .whileTrue(
            Commands.run(
                () -> {
                  //   pivot.setPosition(-0.2);
                  pivot.setPosition(-0.73);
                  flywheel.runVelocity(6500);
                },
                pivot));
    */

    operator
        .x()
        .onTrue(
            FlywheelCommands.aimAmp(pivot, flywheel)
                .andThen(FlywheelCommands.outtakeAmp(indexer, flywheel, pivot)));

    operator.b().onTrue(IntakeCommands.intakeHandoff(intake, indexer, pivot, lightstrip));
    operator.y().onTrue(IntakeCommands.indexSequence(intake, indexer, pivot));
    // controller.x().onTrue(IntakeCommands.bringOutPiece(indexer));
    // operator.leftBumper().onTrue(FlywheelCommands.outtake(indexer, intake));
    // operator.rightBumper().onTrue(FlywheelCommands.outtakeAmp(indexer, flywheel, pivot));

    // operator.rightBumper().onTrue(IntakeCommands.transferPiece(intake, indexer, pivot));

    // controller.leftBumper().onTrue(FlywheelCommands.shoot(indexer));
    // controller.rightBumper().onTrue(Commands.runOnce(() -> indexer.stop()));
    // operator.leftBumper().onTrue(IntakeCommands.transferPiece(intake, indexer));

    // // controller.button(2).onTrue(IntakeCommands.passPieceIntake(intake));

    // controller.
    // controller.axisGreaterThan(2, 0.5).whileTrue(IntakeCommands.runWheelVolts(intake));
    // controller.axisLessThan(2, 0.49).whileTrue(Commands.run(() -> intake.runWheelVolts(0.0)));

    // controller
    //     .a()
    //     .whileTrue(
    //         FlywheelCommands.shootWhileMove(
    //             flywheel, drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));
    // controller
    //     .axisGreaterThan(2, 0.5)
    //     .whileTrue(FlywheelCommands.autoShoot(flywheel, drive, pivot));
    // controller.axisLessThan(2, 0.49).onTrue(Commands.runOnce(flywheel::stop, flywheel));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // PathPlannerPath exampleChorePath = PathPlannerPath.fromChoreoTrajectory("3 Note Auto");
    // return AutoBuilder.followPath(exampleChorePath);
    return autoChooser.get();
  }
}
