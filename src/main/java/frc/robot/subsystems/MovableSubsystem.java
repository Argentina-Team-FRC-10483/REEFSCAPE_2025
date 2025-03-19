package frc.robot.subsystems;

public interface MovableSubsystem {
  void reset();

  double getActualPosition();

  double getTargetPosition();

  void move(double position);
}