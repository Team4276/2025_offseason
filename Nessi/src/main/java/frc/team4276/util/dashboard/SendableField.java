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

  public static void addPoseToBuilder(final SendableBuilder builder, String key, Pose2d pose) {
    builder.addDoubleArrayProperty(
        key, () -> new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()}, null);
  }

  public SendableField withRobot(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;

    return this;
  }

  public void clearRobot() {
    poseSupplier = () -> Pose2d.kZero;
  }

  public SendableField withPath(List<Pose2d> pathSupplier) {
    pathSupplier.forEach(this.pathSupplier.get()::add);

    return this;
  }

  public void clearPath() {
    pathSupplier = () -> List.of();
  }

  public SendableField withTrajectory(Trajectory<SwerveSample> trajectory) {
    List<Pose2d> path = new LinkedList<>();
    for (var pose : trajectory.getPoses()) {
      path.add(pose);
    }

    path.forEach(this.trajectorySupplier.get()::add);

    return this;
  }

  public void clearTrajectory() {
    trajectorySupplier = () -> List.of();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Field");

    if (poseSupplier != null) {
      addPoseToBuilder(builder, "Robot", poseSupplier.get());
    }

    if (pathSupplier != null) {
      int i = 0;
      for (var pose : pathSupplier.get()) {
        addPoseToBuilder(builder, "Path" + i, pose);
        i++;
      }
    }

    if (trajectorySupplier != null) {
      double[] array = new double[trajectorySupplier.get().size() * 3];
      for (int i = 0; i < trajectorySupplier.get().size(); i++) {
        array[3 * i] = trajectorySupplier.get().get(i).getX();
        array[3 * i + 1] = trajectorySupplier.get().get(i).getY();
        array[3 * i + 2] = trajectorySupplier.get().get(i).getRotation().getDegrees();
      }

      builder.addDoubleArrayProperty("Trajectory", () -> array, null);
    }
  }
}
