package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DeadZone;
import frc.robot.subsystems.MunecaSubsystem;
import frc.robot.utils.Utils;

public class MunecaCommand extends Command {
  private final DoubleSupplier munecaPower;
  private final MunecaSubsystem munecaSubsystem;

  public MunecaCommand(MunecaSubsystem munecaSubsystem, DoubleSupplier munecaPower) {
    this.munecaPower = munecaPower;
    this.munecaSubsystem = munecaSubsystem;

    addRequirements(this.munecaSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    munecaSubsystem.moveMuneca(Utils.applyDeadZone(munecaPower.getAsDouble(), DeadZone.ElevadorDeadZone));
  }

  @Override
  public void end(boolean isInterrupted) {
    munecaSubsystem.moveMuneca(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
