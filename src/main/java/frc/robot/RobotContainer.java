/*
 * RobotContainer.java
 * 
 * Este archivo contiene la configuración principal del robot, incluyendo:
 * - Subsistemas
 * - Controladores
 * - Bindings de comandos
 * - Configuración autónoma
 */
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

/////////////////////////////////////////////////////////////
// CLASE PRINCIPAL - ROBOT CONTAINER
/////////////////////////////////////////////////////////////
public class RobotContainer {

  /////////////////////////////////////////////////////////////
  // DECLARACIÓN DE SUBSISTEMAS Y CONTROLADORES
  /////////////////////////////////////////////////////////////
  private final MovementSubsystem movementSubsystem = new MovementSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(armSubsystem);
  private final RodLateralesSubsystem rodLateralesSubsystem = new RodLateralesSubsystem();
  private final HangingSubsystem hangingSubsystem = new HangingSubsystem();
  
  // Controladores Xbox
  private final CommandXboxController driverController = new CommandXboxController(
    OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(
    OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // Selector de autónomo
  private final SendableChooser<Command> autoChooser;

  /////////////////////////////////////////////////////////////
  // CONSTRUCTOR - CONFIGURACIÓN INICIAL
  /////////////////////////////////////////////////////////////
  public RobotContainer() {
    // Registro de comandos nombrados para PathPlanner
    NamedCommands.registerCommand("Elevator to L3", new MoveToPositionCommand(Constants.ElevatorConstants.L3, elevatorSubsystem, 3, true));
    NamedCommands.registerCommand("Elevator to L1", new MoveToPositionCommand(Constants.ElevatorConstants.L1, elevatorSubsystem, 3, true));
    NamedCommands.registerCommand("Elevator to L2", new MoveToPositionCommand(Constants.ElevatorConstants.L2, elevatorSubsystem, 3, true));
    NamedCommands.registerCommand("Elevator to 0", new MoveToPositionCommand(Constants.ElevatorConstants.L0, elevatorSubsystem, 3, true));
    NamedCommands.registerCommand("Drop coral", new MoveToPositionCommand(-11, armSubsystem, 1, true));
    NamedCommands.registerCommand("Lift claw", new MoveToPositionCommand(-1.6, armSubsystem, 1, true));
    NamedCommands.registerCommand("Fix claw", new MoveToPositionCommand(-5, armSubsystem, 1, true));
    NamedCommands.registerCommand("Roll 1s", new SideRodCommandTimed(rodLateralesSubsystem, -0.3, 2));

    // Configuración del selector de autónomo
    autoChooser = AutoBuilder.buildAutoChooser();

    // Configuración de bindings de controles
    configureBindings();

    // Mostrar selector de autónomo en SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /////////////////////////////////////////////////////////////
  // CONFIGURACIÓN DE BINDINGS DE CONTROLES
  /////////////////////////////////////////////////////////////
  private void configureBindings() {
    // Subsistema de enganche (hanging)
    hangingSubsystem.setDefaultCommand(new HangingCommand(
      hangingSubsystem,
      () -> {
        double power = 0;
        if (driverController.x().getAsBoolean()) power += 1;
        if (driverController.b().getAsBoolean()) power -= 1;
        return power;
      }
    ));

    // Subsistema de brazo/muñeca
    AccelIncrementalMoveCommand defaultCommandArm = new AccelIncrementalMoveCommand(
      () -> operatorController.getRightY() * -0.7,
      armSubsystem,
      3.6
    );
    defaultCommandArm.addRequirements(armSubsystem);
    armSubsystem.setDefaultCommand(defaultCommandArm);

    // Subsistema de elevador
    IncrementalMoveCommand defaultCommandElevator = new IncrementalMoveCommand(
      () -> operatorController.getLeftY() * -0.8,
      elevatorSubsystem
    );
    defaultCommandElevator.addRequirements(elevatorSubsystem);
    elevatorSubsystem.setDefaultCommand(defaultCommandElevator);

    // Bindings de botones del operador
    operatorController.leftTrigger().whileTrue(new SideRodCommand(rodLateralesSubsystem, 0.5));
    operatorController.rightTrigger().whileTrue(new SideRodCommand(rodLateralesSubsystem, -0.5));
    operatorController.rightStick().onTrue(new IncreaseArmLimitCommand(armSubsystem, 0.5, 0.5));
    operatorController.leftStick().onTrue(new IncreaseElevatorLimitCommand(elevatorSubsystem, 3, 3));
    
    // Subsistema de movimiento
    movementSubsystem.setDefaultCommand(new MovementCommand(
      () -> -driverController.getLeftY() * 0.5,
      () -> driverController.getHID().getLeftBumperButton(),
      () -> -driverController.getRightX() * 0.4,
      movementSubsystem
    ));
    
    // Setpoints predefinidos del elevador
    if (Constants.ElevatorConstants.ENABLE_ELEVATOR_SETPOINTS) {
      operatorController.povDown().onTrue(new MoveToPositionCommand(Constants.ElevatorConstants.L0, elevatorSubsystem, 3, false)); // Nivel 1
      operatorController.povLeft().onTrue(new MoveToPositionCommand(Constants.ElevatorConstants.L1, elevatorSubsystem, 3, false)); // Nivel 2
      operatorController.povUp().onTrue(new MoveToPositionCommand(Constants.ElevatorConstants.L2, elevatorSubsystem, 3, false)); // Nivel 3
      operatorController.povRight().onTrue(new MoveToPositionCommand(Constants.ElevatorConstants.L3, elevatorSubsystem, 3, false)); // Nivel 4
    }
  }

  /////////////////////////////////////////////////////////////
  // MÉTODO PARA OBTENER EL COMANDO AUTÓNOMO SELECCIONADO
  /////////////////////////////////////////////////////////////
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}