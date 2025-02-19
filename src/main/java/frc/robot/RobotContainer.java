// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.ElevadorCommand;
import frc.robot.commands.EngancheCommand;
import frc.robot.commands.MovimientoCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevadorSubsystem;
import frc.robot.subsystems.EngancheSubsystem;
import frc.robot.subsystems.MovimientoSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final MovimientoSubsystem movimientoSubsystem = new MovimientoSubsystem();

  private final ElevadorSubsystem elevadorSubsystem = new ElevadorSubsystem();

  private final AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();

  // Control del conductor
  private final EngancheSubsystem engancheSubsystem = new EngancheSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);

  // Control del operador
  private final CommandXboxController operadorController = new CommandXboxController(
      OperatorConstants.OPERADOR_CONTROLLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {

    driverController.rightBumper()
        .whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, 0.5));

    driverController.rightTrigger()
        .whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, -0.5));

    driverController.b()
        .whileTrue(new EngancheCommand(engancheSubsystem, 0.5));

    driverController.leftBumper().onTrue(new AutoDriveCommand(movimientoSubsystem, 3));

    driverController.a().onTrue(new AutoDriveCommand(movimientoSubsystem, 0));

    elevadorSubsystem.setDefaultCommand(
        new ElevadorCommand(
            elevadorSubsystem,
            () -> operadorController.getLeftY()));

    driverController.x()
        .whileTrue(new EngancheCommand(engancheSubsystem, -0.5));

    movimientoSubsystem.setDefaultCommand(new MovimientoCommand(
        () -> -driverController.getLeftY() * 0.5,
        () -> driverController.getLeftTriggerAxis() * 0.5,
        () -> -driverController.getRightX() * 0.5,
        movimientoSubsystem));
  }

  public Command getAutonomousCommand() {
    return new AutoDriveCommand(movimientoSubsystem, 5);
  }
}
