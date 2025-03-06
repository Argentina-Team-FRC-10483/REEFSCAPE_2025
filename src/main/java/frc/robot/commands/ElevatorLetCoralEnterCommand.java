package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorLetCoralEnterCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final double targetPosition = 30.0;
  
  public ElevatorLetCoralEnterCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(this.elevatorSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
      double currentPosition = elevatorSubsystem.getElevatorPosition();
      double speed = 0.3;
  
      if (currentPosition < targetPosition) {
          double distanceRemaining = targetPosition - currentPosition;
        
          if (distanceRemaining <= ElevatorSubsystem.SLOWDOWN_RANGE) {
              double slowdownFactor = distanceRemaining / ElevatorSubsystem.SLOWDOWN_RANGE;
              speed *= Math.max(slowdownFactor, 0.05);
          }
      } else if (currentPosition > targetPosition) {
          double distanceRemaining = currentPosition - targetPosition;
  
          if (distanceRemaining <= ElevatorSubsystem.SLOWDOWN_RANGE) {
              double slowdownFactor = distanceRemaining / ElevatorSubsystem.SLOWDOWN_RANGE;
              speed *= Math.max(slowdownFactor, 0.05);
          }
  
          speed = -speed;
      }
  
      if (Math.abs(speed) < 0.05) {
          speed = Math.signum(speed) * 0.05;
      }
  
      elevatorSubsystem.moveElevator(speed);
  }
  

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.moveElevator(0);  // Aseguramos que el elevador se detenga al finalizar
  }

  @Override
  public boolean isFinished() {
      return elevatorSubsystem.getElevatorPosition() >= (targetPosition - 0.5) && elevatorSubsystem.getElevatorPosition() <= (targetPosition + 0.5);
  }
  
}
