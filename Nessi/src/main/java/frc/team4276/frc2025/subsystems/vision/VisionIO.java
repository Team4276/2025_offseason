package frc.team4276.frc2025.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TagObservation[] targetObservations = new TagObservation[0];
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /** Represents the angle to a simple target, used for trig based pose estimation. */
  public static record TagObservation(
      int tagId, double timestamp, int camera, double tx, double distance, Pose2d robotPose) {}

  public static record PoseObservation(
      int[] tagIds, double timestamp, int camera, Pose3d robotPose, double distance) {}

  public default void updateInputs(VisionIOInputs inputs) {}
}
