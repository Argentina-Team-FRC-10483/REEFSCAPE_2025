// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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

  private final MovementSubsystem movementSubsystem = new MovementSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final MunecaSubsystem munecaSubsystem = new MunecaSubsystem();
  RodLateralesSubsystem rodLateralesSubsystem = new RodLateralesSubsystem();
  private final RodInteriorSubsystem rodInteriorSubsystem = new RodInteriorSubsystem();
  private final EngancheSubsystem engancheSubsystem = new EngancheSubsystem();
  private final CommandXboxController driverController = new CommandXboxController(
    OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(
    OperatorConstants.OPERATOR_CONTROLLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    IncrementalMoveCommand defaultCommand = new IncrementalMoveCommand(() -> operatorController.getRightX() * -0.2, engancheSubsystem
    );
    defaultCommand.addRequirements(engancheSubsystem);
      .setDefaultCommand(
      defaultCommand);

    // Muñeca Binding
    IncrementalMoveCommand defaultCommandMuneca = new IncrementalMoveCommand(() -> operatorController.getRightY() * -0.2,
      munecaSubsystem);
    defaultCommandMuneca.addRequirements(munecaSubsystem);
    munecaSubsystem.setDefaultCommand(defaultCommandMuneca);

    // Elevator
    IncrementalMoveCommand defaultCommandElevator = new IncrementalMoveCommand(
      () -> operatorController.getLeftY() * -0.4, elevatorSubsystem);
    defaultCommandElevator.addRequirements(elevatorSubsystem);
    elevatorSubsystem.setDefaultCommand(defaultCommandElevator);

    operatorController.leftTrigger().whileTrue(new RodLateralesCommand(rodLateralesSubsystem, 0.5));
    operatorController.rightTrigger().whileTrue(new RodLateralesCommand(rodLateralesSubsystem, -0.5));
    operatorController.b().whileTrue(new RodInteriorCommand(rodInteriorSubsystem, 0.2));

    movementSubsystem.setDefaultCommand(new MovementCommand(
      () -> -driverController.getLeftY() * 0.5,
      () -> driverController.getHID().getLeftBumperButton(),
      () -> -driverController.getRightX() * 0.3,
      movementSubsystem));

    operatorController.povDown().onTrue(new MoveToPositionCommand(Constants.ElevatorConstants.L0, elevatorSubsystem, 3, false)); // Nivel 1
    operatorController.povLeft().onTrue(new MoveToPositionCommand(Constants.ElevatorConstants.L1, elevatorSubsystem, 3, false)); // Nivel 2
    operatorController.povUp().onTrue(new MoveToPositionCommand(Constants.ElevatorConstants.L2, elevatorSubsystem, 3, false)); // Nivel 3
    operatorController.povRight().onTrue(new MoveToPositionCommand(Constants.ElevatorConstants.L3, elevatorSubsystem, 3, false)); // Nivel 4

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}