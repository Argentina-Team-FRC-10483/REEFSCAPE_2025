package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PinzaConstants;

public class ClawSubsystem extends SubsystemBase{            
    
    private final PIDController pidController; // Controlador PID para la articulación

    private final PWMSparkMax joint;
    private final PWMSparkMax motorLeft;
    private final PWMSparkMax motorRight;
    
    public int clawState;
    private double postion;

    public ClawSubsystem(){
        this.joint = new PWMSparkMax(PinzaConstants.PinzaMotorArticulacionCentral_ID);
        this.motorLeft = new PWMSparkMax(PinzaConstants.PinzaMotorRodilloIzquierdo_ID);
        this.motorRight = new PWMSparkMax(PinzaConstants.PinzaMotorRodilloDerecho_ID);
        this.pidController = new PIDController(PinzaConstants.Kp, PinzaConstants.Ki, PinzaConstants.Kd);
        this.pidController.setTolerance(PinzaConstants.Tolerance);
        this.clawState = 0;
        this.postion = 0.0;
    }

    public void State(){
        clawState = (clawState + 1) % 3; // Alternar entre 0 (abierta), 1 (media) y 2 (cerrada)
        switch (clawState) {
            case 2:
                pidController.setSetpoint(100); // Ir a la posición abierta
                break;
            case 1:
                pidController.setSetpoint(50); // Ir a la posición media
                break;
            case 0:
                pidController.setSetpoint(0); // Ir a la posición cerrada
                break;
        }
    }

    public void aperturaPinza(){
        double PidPut = pidController.calculate(postion);
        joint.set(PidPut);

        postion += PidPut * 0.1;
        System.out.println(postion);
    }

    public void translation (int direction){
        this.motorLeft.set(direction);
        this.motorRight.set(direction);
    }
}