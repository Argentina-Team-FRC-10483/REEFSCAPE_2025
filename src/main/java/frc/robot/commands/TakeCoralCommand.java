package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.MunecaSubsystem;
import frc.robot.subsystems.RodLateralesSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class TakeCoralCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final MunecaSubsystem munecaSubsystem;
  private final RodLateralesSubsystem rodLateralesSubsystem;
  private final Timer timer = new Timer();

  private static final double ELEVATOR_TARGET_POSITION = 20.0;
  private static final double MUNECA_TARGET_POSITION = -10.0;
  private static final double ROD_POWER = 0.5;
  private static final double ROD_DURATION = 2.0;

  public TakeCoralCommand(ElevatorSubsystem elevatorSubsystem, MunecaSubsystem munecaSubsystem, RodLateralesSubsystem rodLateralesSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.munecaSubsystem = munecaSubsystem;
    this.rodLateralesSubsystem = rodLateralesSubsystem;
    addRequirements(this.elevatorSubsystem, this.munecaSubsystem, this.rodLateralesSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    // Mueve el elevador a la posición objetivo
    double elevatorPosition = elevatorSubsystem.getElevatorPosition();
    double elevatorSpeed = 0.3;

    if (elevatorPosition < ELEVATOR_TARGET_POSITION) {
      double distanceRemaining = ELEVATOR_TARGET_POSITION - elevatorPosition;
      if (distanceRemaining <= ElevatorSubsystem.SLOWDOWN_RANGE) {
        double slowdownFactor = distanceRemaining / ElevatorSubsystem.SLOWDOWN_RANGE;
        elevatorSpeed *= Math.max(slowdownFactor, 0.05);
      }
    } else if (elevatorPosition > ELEVATOR_TARGET_POSITION) {
      double distanceRemaining = elevatorPosition - ELEVATOR_TARGET_POSITION;
      if (distanceRemaining <= ElevatorSubsystem.SLOWDOWN_RANGE) {
        double slowdownFactor = distanceRemaining / ElevatorSubsystem.SLOWDOWN_RANGE;
        elevatorSpeed *= Math.max(slowdownFactor, 0.05);
      }
      elevatorSpeed = -elevatorSpeed;
    }

    elevatorSubsystem.moveElevator(elevatorSpeed);

    // Mueve la muñeca a la posición objetivo
    double munecaPosition = munecaSubsystem.getMunecaPosition();
    double munecaSpeed = 0.3;

    if (munecaPosition < MUNECA_TARGET_POSITION) {
      double distanceRemaining = MUNECA_TARGET_POSITION - munecaPosition;
      if (distanceRemaining <= MunecaSubsystem.SLOWDOWN_RANGE) {
        double slowdownFactor = distanceRemaining / MunecaSubsystem.SLOWDOWN_RANGE;
        munecaSpeed *= Math.max(slowdownFactor, 0.05);
      }
      munecaSpeed = -munecaSpeed;
    } else if (munecaPosition > MUNECA_TARGET_POSITION) {
      double distanceRemaining = munecaPosition - MUNECA_TARGET_POSITION;
      if (distanceRemaining <= MunecaSubsystem.SLOWDOWN_RANGE) {
        double slowdownFactor = distanceRemaining / MunecaSubsystem.SLOWDOWN_RANGE;
        munecaSpeed *= Math.max(slowdownFactor, 0.05);
      }
    }

    munecaSubsystem.moveMuneca(munecaSpeed);

    // Mueve los rodillos laterales durante 2 segundos
    if (timer.get() < ROD_DURATION) {
      rodLateralesSubsystem.andarRodillo(ROD_POWER);
    } else {
      rodLateralesSubsystem.andarRodillo(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.moveElevator(0);  // Detén el elevador
    munecaSubsystem.moveMuneca(0);  // Detén la muñeca
    rodLateralesSubsystem.andarRodillo(0);  // Detén los rodillos laterales
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.getElevatorPosition() >= (ELEVATOR_TARGET_POSITION - 0.5) &&
           elevatorSubsystem.getElevatorPosition() <= (ELEVATOR_TARGET_POSITION + 0.5) &&
           munecaSubsystem.getMunecaPosition() >= (MUNECA_TARGET_POSITION - 0.5) &&
           munecaSubsystem.getMunecaPosition() <= (MUNECA_TARGET_POSITION + 0.5) &&
           timer.get() >= ROD_DURATION;
  }
}