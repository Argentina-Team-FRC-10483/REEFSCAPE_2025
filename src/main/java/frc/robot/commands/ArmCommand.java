package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DeadZone;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.MovimientoSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmCommand extends Command {
    
    private final ArmSubsystem AS;

    private final DoubleSupplier yPosition;
    private final SlewRateLimiter Filter = new SlewRateLimiter(0.5);
    private double lastTime;
    private double direcc;
        
    public ArmCommand(DoubleSupplier size, ArmSubsystem armsubsystem){
        yPosition = size;
        AS = armsubsystem; 
        addRequirements(this.AS);
        this.lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double Posicion = yPosition.getAsDouble(); 
        double gearRatio = 45.0;
        // double maxAngle = 180;
        double deltaTime = Timer.getFPGATimestamp() - lastTime;

        Posicion = aplicarDeadZone(Posicion, DeadZone.BrazoDeadZone);
        AS.getArmAngle();

        if (Posicion == 0.0){
            this.direcc += 0 ;
        }else if (Posicion > 0.0){
            this.direcc += .15 * deltaTime;
        }else if (Posicion < 0.0){
            this.direcc -= .15 * deltaTime;
        }

        

        lastTime = Timer.getFPGATimestamp();

        double angle = Posicion * 180; // Convertir a ángulo (-90° a 90°)
        AS.setArmAngleSimulate(angle);
        
        AS.armMove(AS.setArmAngle(Posicion));
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

    @Override
    public void end(boolean interrupted) {
    }
}
