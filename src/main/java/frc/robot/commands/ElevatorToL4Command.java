package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorToL4Command extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private static final double TARGET_ROTATIONS = 30.0;
  private static final double POWER = 0.5;

  public ElevatorToL4Command(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(this.elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.resetEncoder();
  }

  @Override
  public void execute() {
    if (elevatorSubsystem.getElevatorPosition() < TARGET_ROTATIONS) {
      elevatorSubsystem.moveElevator(POWER);
    } else {
      elevatorSubsystem.moveElevator(0);
    }
  }

  @Override
  public void end(boolean isInterrupted) {
    elevatorSubsystem.moveElevator(0);
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.getElevatorPosition() >= TARGET_ROTATIONS;
  }
}
