package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DeadZone;
import frc.robot.subsystems.EyesSubsystem;
import frc.robot.subsystems.MovementSubsystem;

import java.lang.reflect.Array;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class MovimientoCommand extends Command {
  private final DoubleSupplier xSpeed;
  private final DoubleSupplier zRotation;
  private final DoubleSupplier aceleracion;
  private final MovementSubsystem driveSubsystem;

  public MovimientoCommand(
      DoubleSupplier xSpeed, DoubleSupplier aceleracion, DoubleSupplier zRotation, MovementSubsystem driveSubsystem) {
    this.xSpeed = xSpeed;
    this.aceleracion = aceleracion;
    this.zRotation = zRotation;
    this.driveSubsystem = driveSubsystem;

    addRequirements(this.driveSubsystem);
    
  }

  // Runs each time the command is scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Giro", 0);
    SmartDashboard.putNumber("Velocidad", 0);
    SmartDashboard.putNumber("Aceleracion", 0);
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    double velocidad = xSpeed.getAsDouble();
    double acel = aceleracion.getAsDouble();
    double giro = zRotation.getAsDouble();

    velocidad = aplicarDeadZone(velocidad, DeadZone.MovimientoDeadZone);
    giro = aplicarDeadZone(giro, DeadZone.MovimientoDeadZone);

    SmartDashboard.putNumber("Giro", giro);
    SmartDashboard.putNumber("Velocidad", velocidad);
    SmartDashboard.putNumber("Aceleracion", acel);


    if (velocidad > 0){
      velocidad = velocidad + acel;
    } else if (velocidad != 0){
      velocidad = velocidad - acel;
    }

    if (giro > 0){
      giro = giro + acel;
    } else if (giro != 0){
      giro = giro - acel;
    } 

    driveSubsystem.driveArcade(velocidad, giro);
  }
  public double aplicarDeadZone(double input, double deadZone) {
    double output = input;
        if (input <= deadZone && input > 0) {
      output = 0;
    } else if(input >= -deadZone && input < 0) {
      output = 0;
    }
    return output;
  }
  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // Return false to indicate that this command never ends. It can be interrupted
    // by another command needing the same subsystem.
    return false;
  }
}
