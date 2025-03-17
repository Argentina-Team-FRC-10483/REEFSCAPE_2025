// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
   private final EngancheSubsystem engancheSubsystem = new EngancheSubsystem();
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(
      OperatorConstants.OPERATOR_CONTROLLER_PORT);

    private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("Elevator to L3", new MoveToPositionCommand(Constants.ElevatorConstants.L3, elevatorSubsystem, 3, true));
    NamedCommands.registerCommand("Elevator to L1", new MoveToPositionCommand(Constants.ElevatorConstants.L1, elevatorSubsystem, 3, true));
    NamedCommands.registerCommand("Elevator to L2", new MoveToPositionCommand(Constants.ElevatorConstants.L2, elevatorSubsystem, 3, true));
    NamedCommands.registerCommand("Elevator to 0", new MoveToPositionCommand(Constants.ElevatorConstants.L0, elevatorSubsystem,3, true ));
    NamedCommands.registerCommand("Drop coral", new MoveToPositionCommand(-11, munecaSubsystem, 1, true));
    NamedCommands.registerCommand("Lift claw", new MoveToPositionCommand(-1.6, munecaSubsystem, 1, true));
    NamedCommands.registerCommand("Fix claw", new MoveToPositionCommand(-5, munecaSubsystem, 1, true));

     autoChooser = AutoBuilder.buildAutoChooser();


    configureBindings();

    SmartDashboard.putData("Auto Chooser", autoChooser);
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
   * PS4} controllers 
   * or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // enganche
      engancheSubsystem.setDefaultCommand(new EngancheCommand(engancheSubsystem,
      () -> {
      double power = 0;
      if (driverController.x().getAsBoolean()) power += 0.1;
      if (driverController.b().getAsBoolean()) power -= 0.1;
      return power;
      }
      ));

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

    movementSubsystem.setDefaultCommand(new MovementCommand(
        () -> -driverController.getLeftY() * 0.5,
        () -> driverController.getHID().getLeftBumperButton(),
        () -> -driverController.getRightX() * 0.3,
        movementSubsystem));

    //operatorController.povDown().onTrue(new MoveToPositionCommand(Constants.ElevatorConstants.L0, elevatorSubsystem, 3, false)); // Nivel 1
    //operatorController.povLeft().onTrue(new MoveToPositionCommand(Constants.ElevatorConstants.L1, elevatorSubsystem, 3, false)); // Nivel 2
    //operatorController.povUp().onTrue(new MoveToPositionCommand(Constants.ElevatorConstants.L2, elevatorSubsystem, 3, false)); // Nivel 3
    //operatorController.povRight().onTrue(new MoveToPositionCommand(Constants.ElevatorConstants.L3, elevatorSubsystem, 3, false)); // Nivel 4
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}