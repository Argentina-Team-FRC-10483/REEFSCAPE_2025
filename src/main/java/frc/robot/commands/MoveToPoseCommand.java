package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MovementSubsystem;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.List;

public class MoveToPoseCommand extends Command {
  private final MovementSubsystem movementSubsystem;
  private final Pose2d targetPose;
  private final LTVUnicycleController controller;
  private Trajectory trajectory;
  private double startTime = 0;
  StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance
    .getDefault()
    .getStructArrayTopic("Target/Path", Pose2d.struct).publish();


  public MoveToPoseCommand(Pose2d targetPose, MovementSubsystem movementSubsystem) {
    this.targetPose = targetPose;
    this.movementSubsystem = movementSubsystem;
    addRequirements(movementSubsystem);
    controller = new LTVUnicycleController(
      VecBuilder.fill(0.0625, 0.0625, 0.5),
      VecBuilder.fill(1.0, 1.0),
      0.02
    );
  }

  public void computeTrajectory() {
    TrajectoryConfig config = new TrajectoryConfig(2.0, 1.0);
    config.addConstraint(new CentripetalAccelerationConstraint(0.1));
    trajectory = TrajectoryGenerator.generateTrajectory(
      movementSubsystem.getPose(),
      List.of(),
      targetPose,
      config
    );
    arrayPublisher.set(trajectory.getStates().stream().map(x->x.poseMeters).toArray(Pose2d[]::new));
  }

  public double getTime() {
    return (double) System.currentTimeMillis() / 1000d;
  }

  @Override
  public void execute() {
    double time = getTime() - startTime;
    movementSubsystem.driveRobotRelative(
      controller.calculate(movementSubsystem.getPose(), trajectory.sample(time))
    );
  }

  @Override
  public boolean isFinished() {
    return getTime() - startTime > trajectory.getTotalTimeSeconds();
  }

  @Override
  public void initialize() {
    startTime = getTime();
    computeTrajectory();
  }
}
