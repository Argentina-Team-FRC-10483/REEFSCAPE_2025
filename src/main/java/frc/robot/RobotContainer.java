// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
//  private final SendableChooser<Command> autoChooser;
  private final MovementSubsystem movementSubsystem = new MovementSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final MunecaSubsystem munecaSubsystem = new MunecaSubsystem();
  RodLateralesSubsystem rodLateralesSubsystem = new RodLateralesSubsystem();
  private final RodInteriorSubsystem rodInteriorSubsystem = new RodInteriorSubsystem();
//  private final EngancheSubsystem engancheSubsystem = new EngancheSubsystem();
  private final CommandXboxController driverController =
    new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
    new CommandXboxController(OperatorConstants.OPERADOR_CONTROLLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
//     NamedCommands.registerCommand("Elevator to L4", new MoveToPositionCommand(40,elevatorSubsystem, 3));
//     NamedCommands.registerCommand("Elevator to L3", new MoveToPositionCommand(30,elevatorSubsystem, 3));
//     NamedCommands.registerCommand("Elevator to L2", new ElevatorToPositionCommand(elevatorSubsystem, 20));
//     NamedCommands.registerCommand("Elevator to L1", new ElevatorToPositionCommand(elevatorSubsystem, 10));
//     NamedCommands.registerCommand("RollerIntakeClaw", new RodLatTakeCoralCommand(rodLateralesSubsystem, 0.1, 2));
//     NamedCommands.registerCommand("RollerScoreCoral", new TimedRollerScoreCommand(rodInteriorSubsystem, 0.1, 2));
//     NamedCommands.registerCommand("Wrist Position L4", new MunecaToPositionCommand(munecaSubsystem, -2));
//     NamedCommands.registerCommand("Wrist Position L3, 2 and 1", new MunecaToPositionCommand(munecaSubsystem, -4));
//     NamedCommands.registerCommand("Wrist Position Grab Coral", new MunecaToPositionCommand(munecaSubsystem, -15));
//    autoChooser = AutoBuilder.buildAutoChooser();
    configureBindings();
//    SmartDashboard.putData("Auto Chooser", autoChooser);
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

//    engancheSubsystem.setDefaultCommand(new EngancheCommand(engancheSubsystem,
//      () -> {
//        double power = 0;
//        if (driverController.x().getAsBoolean()) power += 0.5;
//        if (driverController.b().getAsBoolean()) power -= 0.5;
//        return power;
//      }
//    ));
    IncrementalMoveCommand defaultCommanda = new IncrementalMoveCommand(() -> operatorController.getRightY() * -0.2, munecaSubsystem);
    defaultCommanda.addRequirements(munecaSubsystem);
    munecaSubsystem.setDefaultCommand(defaultCommanda);
    IncrementalMoveCommand defaultCommandb = new IncrementalMoveCommand(() -> operatorController.getLeftY() * -0.4, elevatorSubsystem);
    defaultCommandb.addRequirements(elevatorSubsystem);
    elevatorSubsystem.setDefaultCommand(defaultCommandb);

    operatorController.a().onTrue(new MoveToPositionCommand(-10, munecaSubsystem, 1)); // Position debe ser 3 numeros menor que el valor que queres llegar, por ejemplo si queres que llegue a -11 hay que darle -8

    movementSubsystem.setDefaultCommand(new MovementCommand(
      () -> -driverController.getLeftY() * 0.5,
      () -> driverController.leftBumper().getAsBoolean(),
      () -> -driverController.getRightX() * 0.5,
      movementSubsystem)
    );

operatorController.povDown().onTrue(new MoveToPositionCommand(10, elevatorSubsystem, 3)); // Nivel 1
operatorController.povLeft().onTrue(new MoveToPositionCommand(20, elevatorSubsystem, 3)); // Nivel 2
operatorController.povUp().onTrue(new MoveToPositionCommand(30, elevatorSubsystem, 3)); // Nivel 3
operatorController.povRight().onTrue(new MoveToPositionCommand(40, elevatorSubsystem, 3)); // Nivel 4

//    operatorController.y().onTrue(
//      new ElevatorToPositionCommand(elevatorSubsystem, 30)
//        .andThen(new MunecaToPositionCommand(munecaSubsystem, -2))
//        .andThen(new SequentialCommandGroup(
//          new RodLatTakeCoralCommand(rodLateralesSubsystem, 0.1, 2)
//        ))
//    );

//    operatorController.x().onTrue(
//      new ElevatorToPositionCommand(elevatorSubsystem, 40)
//        .andThen(new MunecaToPositionCommand(munecaSubsystem, -2))
//        .andThen(new SequentialCommandGroup(
//          new TimedRollerScoreCommand(rodInteriorSubsystem, 0.1, 2)
//        ))
//    );

operatorController.leftTrigger().whileTrue(new RodLateralesCommand(rodLateralesSubsystem, 0.5));
operatorController.rightTrigger().whileTrue(new RodLateralesCommand(rodLateralesSubsystem, -0.5));
operatorController.a().whileTrue(new RodInteriorCommand(rodInteriorSubsystem, 0.5));


  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
//  public Command getAutonomousCommand() {
//    return autoChooser.getSelected();
//  }
}