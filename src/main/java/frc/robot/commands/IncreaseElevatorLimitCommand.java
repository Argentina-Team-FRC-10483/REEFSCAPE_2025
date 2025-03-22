package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class IncreaseElevatorLimitCommand extends Command {
  ElevatorSubsystem elevatorSubsystem;
  private final double extraUp;
  private final double extraDown;

  public IncreaseElevatorLimitCommand(ElevatorSubsystem elevatorSubsystem, double extraUp, double extraDown) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.extraUp = extraUp;
    this.extraDown = extraDown;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    ElevatorSubsystem.UPPER_LIMIT +=  extraUp;
    ElevatorSubsystem.LOWER_LIMIT -= extraDown;
    ElevatorSubsystem.RESTRICTED_LOWER_LIMIT -= extraDown;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
