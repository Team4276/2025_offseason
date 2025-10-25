package frc.team4276.util.dashboard;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

public class SendableField implements Sendable {
  private Supplier<Pose2d> poseSupplier = () -> Pose2d.kZero;
  private Supplier<List<Pose2d>> pathSupplier = () -> List.of();
  private Supplier<List<Pose2d>> trajectorySupplier = () -> List.of();

  public SendableField() {}

  public SendableField withRobot(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;

    return this;
  }

  public SendableField withPath(Supplier<List<Pose2d>> pathSupplier) {
    this.trajectorySupplier = pathSupplier;

    return this;
  }

  public SendableField withTrajectory(Trajectory<SwerveSample> trajectory) {
    List<Pose2d> path = new LinkedList<>();
    for (var pose : trajectory.getPoses()) {
      path.add(pose);
    }

    this.trajectorySupplier = () -> path;

    return this;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Field");

    if (poseSupplier != null) {
      builder.addDoubleArrayProperty(
          "Robot",
          () ->
              new double[] {
                poseSupplier.get().getX(),
                poseSupplier.get().getY(),
                poseSupplier.get().getRotation().getDegrees()
              },
          null);
    }

    if (trajectorySupplier != null) {
      double[] array = new double[trajectorySupplier.get().size() * 3];
      for (int i = 0; i < trajectorySupplier.get().size(); i++) {
        array[3 * i] = trajectorySupplier.get().get(i).getX();
        array[3 * i + 1] = trajectorySupplier.get().get(i).getY();
        array[3 * i + 2] = trajectorySupplier.get().get(i).getRotation().getDegrees();
      }

      builder.addDoubleArrayProperty("Path", () -> array, null);
    }
  }
}
