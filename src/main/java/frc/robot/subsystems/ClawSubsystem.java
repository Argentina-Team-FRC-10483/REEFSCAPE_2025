package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PinzaConstants;

public class ClawSubsystem extends SubsystemBase{            
    
    private final PIDController pidController; // Controlador PID para la articulación

    private final SparkMax joint;
    private final SparkMax motorLeft;
    private final SparkMax motorRight;
    
    public int clawState;
    private double postion;

    public ClawSubsystem(){
        this.joint = new SparkMax(PinzaConstants.PinzaMotorArticulacionCentral_ID, MotorType.kBrushless);
        this.motorLeft = new SparkMax(PinzaConstants.PinzaMotorRodilloIzquierdo_ID, MotorType.kBrushless);
        this.motorRight = new SparkMax(PinzaConstants.PinzaMotorRodilloDerecho_ID, MotorType.kBrushless);
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