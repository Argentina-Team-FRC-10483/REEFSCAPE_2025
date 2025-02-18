package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase{

    private final SparkMax motorShoulder;
    private final RelativeEncoder encoder;
    private long debugTimeArmSub = 0;

    // Simulacion 
    private final Mechanism2d mechanism;
    private final MechanismRoot2d root;
    private final MechanismLigament2d arm;


    private double armAngle = 0; // Ángulo del brazo

    public ArmSubsystem(){
        //Motor
        this.motorShoulder = new SparkMax(ArmConstants.ArmMotorShoulder_ID, MotorType.kBrushless);
        // Encoder
        this.encoder = motorShoulder.getEncoder(); 
        double Posicion = this.encoder.getPosition();
        double pulsos = Posicion * 42;
        //Configuraciones
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40, 1,5676);
        config.voltageCompensation(12);

         // Simulacion 
         // Crear la simulación del brazo
        mechanism = new Mechanism2d(100, 100); // Tamaño de la simulación en píxeles
        root = mechanism.getRoot("Base", 50, 50); // Posición de la base en el canvas

        // Crear el brazo como un ligamento (nombre, longitud, ángulo, grosor, color)
        arm = root.append(new MechanismLigament2d("Arm", 50, armAngle, 10, new Color8Bit(Color.kYellow)));


        // Publicar en SmartDashboard
        SmartDashboard.putData("Brazo", mechanism);
    }

    public double getArmAngle() {
        return encoder.getPosition();
    }

    public double setArmAngle(double angle) {
        encoder.setPosition(angle);
                return angle;
    }

    public void setArmAngleSimulate(double angle) {
        armAngle = angle;
        arm.setAngle(angle); // Actualizar la simulación
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Pocision Subsystem", getArmAngle());

        // Simulacion 
        SmartDashboard.putData("Brazo", mechanism);
    }

    public void armMove(double torque){
        motorShoulder.set(torque);
    }
}