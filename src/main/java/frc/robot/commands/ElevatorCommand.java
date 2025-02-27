package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DeadZone;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.Utils;

public class ElevatorCommand extends Command {
  private final DoubleSupplier elevatorPower;
  private final ElevatorSubsystem elevatorSubsystem;

  public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier elevatorPower) {
    this.elevatorPower = elevatorPower;
    this.elevatorSubsystem = elevatorSubsystem;

    addRequirements(this.elevatorSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    elevatorSubsystem.moveElevator(Utils.applyDeadZone(-elevatorPower.getAsDouble(), DeadZone.ElevadorDeadZone));
  }

  @Override
  public void end(boolean isInterrupted) {
    elevatorSubsystem.moveElevator(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
