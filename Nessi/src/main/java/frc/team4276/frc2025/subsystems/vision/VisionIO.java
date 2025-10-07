package frc.team4276.frc2025.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation[] targetObservations = new TargetObservation[0];
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /** Represents the angle to a simple target, used for trig based pose estimation. */
  public static record TargetObservation(
      double timestamp, int tagId, int camera, double tx, double distance) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      int tagCount,
      Pose3d fieldToCam1,
      Pose3d fieldToCam2,
      double ambiguity,
      double avgTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    PHOTONVISION
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
