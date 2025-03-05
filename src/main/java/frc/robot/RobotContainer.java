// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.ElevadorCommand;
import frc.robot.commands.EngancheCommand;
import frc.robot.commands.MovimientoCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevadorSubsystem;
import frc.robot.subsystems.EngancheSubsystem;
import frc.robot.subsystems.MovimientoSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
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
    OperatorConstants.DRIVER_CONTROLLER_PORT
  );
  private final CommandXboxController operatorController = new CommandXboxController(
    OperatorConstants.OPERADOR_CONTROLLER_PORT
  );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverController.rightBumper().whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, 0.5));
    driverController.rightTrigger().whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, -0.5));

    engancheSubsystem.setDefaultCommand(new EngancheCommand(engancheSubsystem, 
    () -> {
      double power = 0;
      if (driverController.x().getAsBoolean()) power += 0.5;
      if (driverController.b().getAsBoolean()) power -= 0.5;
      return power;
    }
  ));
  

    elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem, () -> operatorController.getLeftY() * 0.3));
    munecaSubsystem.setDefaultCommand(new MunecaCommand(munecaSubsystem, () -> operatorController.getRightY() * 0.5));

    operatorController.leftTrigger().whileTrue(new RodLateralesCommand(rodLateralesSubsystem, 0.5));
    operatorController.rightTrigger().whileTrue(new RodLateralesCommand(rodLateralesSubsystem, -0.5));
    operatorController.a().whileTrue(new RodInteriorCommand(rodInteriorSubsystem, 0.5));

    movementSubsystem.setDefaultCommand(new MovementCommand(
      () -> -driverController.getLeftY() * 0.5,
      () -> driverController.getHID().getLeftBumperButton(),
      () -> -driverController.getRightX() * 0.5,
      movementSubsystem
    ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    

   driverController.rightBumper()
   .whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, 0.5));

   driverController.rightTrigger()
   .whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, -0.5));

   driverController.b()
   .whileTrue(new EngancheCommand(engancheSubsystem, 0.5));


   elevadorSubsystem.setDefaultCommand(
        new ElevadorCommand(
        elevadorSubsystem,
    () ->  operadorController.getLeftY()));

   driverController.x()
   .whileTrue(new EngancheCommand(engancheSubsystem, -0.5));

  movimientoSubsystem.setDefaultCommand(new MovimientoCommand(
    () -> -driverController.getLeftY()*0.5,
    () ->  driverController.getLeftTriggerAxis()*0.5,
    () -> -driverController.getRightX()*0.5,
    movimientoSubsystem));

  }

  public Command getAutonomousCommand() {
    return null;
  }
}