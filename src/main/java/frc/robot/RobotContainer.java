// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorToL4Command;
import frc.robot.commands.EngancheCommand;
import frc.robot.commands.MovementCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EngancheSubsystem;
import frc.robot.subsystems.MovementSubsystem;

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
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
  private final EngancheSubsystem engancheSubsystem = new EngancheSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(
    OperatorConstants.DRIVER_CONTROLLER_PORT
  );
  private final CommandXboxController operatorController = new CommandXboxController(
    OperatorConstants.OPERADOR_CONTROLLER_PORT
  );

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("SubirElevador", new ElevatorToL4Command(elevatorSubsystem));

    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    driverController.rightBumper().whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, 0.5));
    driverController.rightTrigger().whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, -0.5));
    driverController.b().whileTrue(new EngancheCommand(engancheSubsystem, 0.5));
    driverController.x().whileTrue(new EngancheCommand(engancheSubsystem, -0.5));

    elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem, () -> operatorController.getLeftY() * -0.5));
    movementSubsystem.setDefaultCommand(new MovementCommand(
      () -> -driverController.getLeftY() * 0.5,
      () -> driverController.getHID().getLeftBumperButton(),
      () -> -driverController.getRightX() * 0.5,
      movementSubsystem
    ));

  driverController.povUp().whileTrue(movementSubsystem.sysIdQuasistatic(Direction.kForward));
  driverController.povRight().whileTrue(movementSubsystem.sysIdQuasistatic(Direction.kReverse));
  driverController.povDown().whileTrue(movementSubsystem.sysIdDynamic(Direction.kForward));
  driverController.povLeft().whileTrue(movementSubsystem.sysIdDynamic(Direction.kReverse));
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