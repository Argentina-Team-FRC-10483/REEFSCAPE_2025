package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class IncreaseArmLimitCommand extends Command {
  ArmSubsystem armSubsystem;
  private final double extraUp;
  private final double extraDown;

  public IncreaseArmLimitCommand(ArmSubsystem armSubsystem, double extraUp, double extraDown) {
    this.armSubsystem = armSubsystem;
    this.extraUp = extraUp;
    this.extraDown = extraDown;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    ArmSubsystem.UPPER_LIMIT +=  extraUp;
    ArmSubsystem.LOWER_LIMIT -= extraDown;
    ElevatorSubsystem.ARM_THRESHOLD -= extraDown;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
