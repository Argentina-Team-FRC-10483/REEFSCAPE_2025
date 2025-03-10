package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DeadZone;
import frc.robot.utils.Utils;
import frc.robot.subsystems.MunecaSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MunecaCommand extends Command {
  private final DoubleSupplier munecaPower;
  private final MunecaSubsystem munecaSubsystem;
  private double lastTime;

  public MunecaCommand(MunecaSubsystem munecaSubsystem, DoubleSupplier munecaPower) {
    this.munecaPower = munecaPower;
    this.munecaSubsystem = munecaSubsystem;
    this.lastTime = Timer.getFPGATimestamp();

    addRequirements(this.munecaSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double deltaTime = Timer.getFPGATimestamp() - this.lastTime;
    
    double input = Utils.applyDeadZone(munecaPower.getAsDouble(), 0.30) * -1.0;
    input = input < 0 ? -1.0 : input == 0 ? 0.0 : 1.0;
    double angle = input * 10 * deltaTime;
    munecaSubsystem.sumAngleDesired(angle);
    SmartDashboard.putNumber("AngleCommand", angle);

    this.lastTime = Timer.getFPGATimestamp();
    //munecaSubsystem.moveMuneca(Utils.applyDeadZone(-munecaPower.getAsDouble(), DeadZone.ElevadorDeadZone));
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
