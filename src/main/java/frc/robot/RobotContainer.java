// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EngancheSubsystem;
import frc.robot.subsystems.EyesSubsystem;
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
  // The robot's subsystems and commands are defined here...

  private final EyesSubsystem eyesSubsystem = new EyesSubsystem();

  private final MovementSubsystem movementSubsystem = new MovementSubsystem();

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();

  private final ClawSubsystem ClawSubsystem = new ClawSubsystem();

  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  // // // Control del conductor
  private final EngancheSubsystem engancheSubsystem = new EngancheSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(
    OperatorConstants.DRIVER_CONTROLLER_PORT
  );
  private final CommandXboxController operatorController = new CommandXboxController(
    OperatorConstants.OPERADOR_CONTROLLER_PORT
  );

  SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    driverController.rightBumper().whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, 0.5));
    driverController.rightTrigger().whileTrue(new AlgaeIntakeCommand(algaeIntakeSubsystem, -0.5));
    driverController.b().whileTrue(new EngancheCommand(engancheSubsystem, 0.5));
    driverController.x().whileTrue(new EngancheCommand(engancheSubsystem, -0.5));

    elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem, () -> operatorController.getLeftY() * 0.5));
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
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}