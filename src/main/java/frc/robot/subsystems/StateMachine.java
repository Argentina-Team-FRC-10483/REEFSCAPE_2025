package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constElevator;
import frc.robot.commands.states.PrepCoralLv;

public class StateMachine extends SubsystemBase {
  public static RobotState currentRobotState;
  public static TargetState currentTargetState;
  ElevadorSubsystem subElevator;
  StateMachine subStateMachine = this;

  /** Creates a new StateMachine. */
  public StateMachine(ElevadorSubsystem subElevator) {
    currentRobotState = RobotState.NONE;
    currentTargetState = TargetState.NONE;

    this.subElevator = subElevator;
  }

  public void setRobotState(RobotState robotState) {
    currentRobotState = robotState;
  }

  public void setTargetState(TargetState targetState) {
    currentTargetState = targetState;
  }

  public RobotState getRobotState() {
    return currentRobotState;
  }

  public TargetState getTargetState() {
    return currentTargetState;
  }

  public Command tryState(RobotState desiredState) {
    switch (desiredState) {
      case PREP_CORAL_L1:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
            return new PrepCoralLv(subStateMachine, subElevator, constElevator.CORAL_L1_HEIGHT);
        }
        break;

      case PREP_CORAL_L2:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
            return new PrepCoralLv(subStateMachine, subElevator, constElevator.CORAL_L2_HEIGHT);
        }
        break;

      case PREP_CORAL_L3:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L4:
            return new PrepCoralLv(subStateMachine, subElevator, constElevator.CORAL_L3_HEIGHT);
        }
        break;

      case PREP_CORAL_L4:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
            return new PrepCoralLv(subStateMachine, subElevator, constElevator.CORAL_L4_HEIGHT);
        }
        break;
    }
    return Commands.print("Invalid State Provided for Elevator (╯‵□′)╯︵┻━┻");
  }

  public static enum RobotState {
    NONE,
    HAS_CORAL,
    PREP_CORAL_L1,
    PREP_CORAL_L2,
    PREP_CORAL_L3,
    PREP_CORAL_L4
  }

  public static enum TargetState {
    NONE,
    PREP_CORAL_L1,
    PREP_CORAL_L2,
    PREP_CORAL_L3,
    PREP_CORAL_L4
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("CURRENT ROBOT STATE", getRobotState().toString());
    SmartDashboard.putString("CURRENT TARGET STATE", getTargetState().toString());
  }
}
