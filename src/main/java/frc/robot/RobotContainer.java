// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private final SideRodSubsystem sideRodSubsystem = new SideRodSubsystem();
  private final InteriorRodSubsystem interiorRodSubsystem = new InteriorRodSubsystem();
  private final EngancheSubsystem engancheSubsystem = new EngancheSubsystem();
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
  }

  public void subsystemDefault(Command command, SubsystemBase subsystem) {
    command.addRequirements(subsystem);
    subsystem.setDefaultCommand(command);
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
    // TODO: This binding is not ideal, we can move it to R1 or L1 maybe?
    subsystemDefault(
      new IncrementalMoveCommand(() -> operatorController.getRightX() * -0.2, engancheSubsystem),
      engancheSubsystem
    );

    // Muñeca Binding
    subsystemDefault(
      new IncrementalMoveCommand(() -> operatorController.getRightY() * -0.2, munecaSubsystem),
      munecaSubsystem
    );

    subsystemDefault(
      new IncrementalMoveCommand(() -> operatorController.getLeftY() * -0.4, elevatorSubsystem),
      elevatorSubsystem
    );

    subsystemDefault(
      new MovementCommand(
        () -> -driverController.getLeftY() * 0.5,
        () -> driverController.getHID().getLeftBumperButton(),
        () -> -driverController.getRightX() * 0.3,
        movementSubsystem
      ),
      movementSubsystem
    );

    operatorController.leftTrigger().onTrue(new MoveToTargetCommand(1, sideRodSubsystem, 0.1, false));
    operatorController.rightTrigger().onTrue(new MoveToTargetCommand(-1, sideRodSubsystem, 0.1, false));
    operatorController.b().onTrue(new MoveToTargetCommand(0.2, interiorRodSubsystem, 0.05, false));


    operatorController.povDown().onTrue(new MoveToTargetCommand(Constants.ElevatorConstants.L0, elevatorSubsystem, 3, false));
    operatorController.povLeft().onTrue(new MoveToTargetCommand(Constants.ElevatorConstants.L1, elevatorSubsystem, 3, false));
    operatorController.povUp().onTrue(new MoveToTargetCommand(Constants.ElevatorConstants.L2, elevatorSubsystem, 3, false));
    operatorController.povRight().onTrue(new MoveToTargetCommand(Constants.ElevatorConstants.L3, elevatorSubsystem, 3, false));
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