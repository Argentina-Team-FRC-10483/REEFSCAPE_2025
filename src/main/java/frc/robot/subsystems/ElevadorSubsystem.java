package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.mapElevator;
import frc.robot.Constants.constElevator;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevadorSubsystem extends SubsystemBase {
    private TalonFX MotorElevadorIzquierdoSeguidor;
    private TalonFX MotorElevadorDerechoLider;
  
    /** Creates a new Elevator. */
    public ElevadorSubsystem() {
    MotorElevadorIzquierdoSeguidor = new TalonFX(mapElevator.MOTOR_ELEVADOR_IZQUIERDO_SEGUIDOR_CAN);
    MotorElevadorDerechoLider = new TalonFX(mapElevator.MOTOR_ELEVADOR_DERECHO_LIDER_CAN);
  
      configure();
    }
  
    public void configure() {
      MotorElevadorDerechoLider.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
      MotorElevadorIzquierdoSeguidor.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    }
    public Distance getElevatorPosition() {
        return Units.Inches.of(MotorElevadorDerechoLider.get());
      }
    
      public void setPosition(Distance height) {
        MotorElevadorDerechoLider.setControl(new PositionVoltage(height.in(Units.Inches)));
        MotorElevadorIzquierdoSeguidor.setControl(new Follower(MotorElevadorDerechoLider.getDeviceID(), true));
      }
    
      public void setNeutral() {
        MotorElevadorDerechoLider.setControl(new NeutralOut());
        MotorElevadorIzquierdoSeguidor.setControl(new NeutralOut());
      }
    
      public void resetSensorPosition(double setpoint) {
        MotorElevadorDerechoLider.setPosition(setpoint);
        MotorElevadorIzquierdoSeguidor.setPosition(setpoint);
    
      }

      @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator/Left/Pos", MotorElevadorIzquierdoSeguidor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/CLO", MotorElevadorIzquierdoSeguidor.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/Output", MotorElevadorIzquierdoSeguidor.get());
    SmartDashboard.putNumber("Elevator/Left/Inverted", MotorElevadorIzquierdoSeguidor.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/Current", MotorElevadorIzquierdoSeguidor.getSupplyCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Elevator/Right/Pos", MotorElevadorDerechoLider.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/CLO", MotorElevadorDerechoLider.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/Output", MotorElevadorDerechoLider.get());
    SmartDashboard.putNumber("Elevator/Right/Inverted", MotorElevadorDerechoLider.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/Current", MotorElevadorDerechoLider.getSupplyCurrent().getValueAsDouble());

  }
    }