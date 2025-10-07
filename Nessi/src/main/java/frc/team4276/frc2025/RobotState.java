package frc.team4276.frc2025;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.team4276.frc2025.field.FieldConstants;
import frc.team4276.frc2025.field.FieldConstants.ReefSide;
import frc.team4276.frc2025.subsystems.vision.VisionConstants;
import frc.team4276.frc2025.subsystems.vision.VisionIO.TagObservation;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.dashboard.ElasticUI;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import frc.team4276.util.pose.VikSwervePoseEstimator;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private LoggedTunableNumber txTyObservationStaleSecs =
      new LoggedTunableNumber("RobotState/TxTyObsStaleSecs", 0.5);
  private static final LoggedTunableNumber minDistanceTagPoseBlend =
      new LoggedTunableNumber("RobotState/MinDistanceTagPoseBlend", Units.inchesToMeters(24.0));
  private static final LoggedTunableNumber maxDistanceTagPoseBlend =
      new LoggedTunableNumber("RobotState/MaxDistanceTagPoseBlend", Units.inchesToMeters(36.0));
  private static final LoggedTunableNumber maxTagAutoSelectDistance =
      new LoggedTunableNumber("RobotState/MaxTagAutoSelectDistance", 1.5);

  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d lastGyroAngle = Rotation2d.kZero;

  private VikSwervePoseEstimator poseEstimator =
      new VikSwervePoseEstimator(kinematics, lastGyroAngle, lastWheelPositions, Pose2d.kZero);
  private VikSwervePoseEstimator poseEstimatorOdom =
      new VikSwervePoseEstimator(kinematics, lastGyroAngle, lastWheelPositions, Pose2d.kZero);
  private VikSwervePoseEstimator poseEstimatorTxty =
      new VikSwervePoseEstimator(kinematics, lastGyroAngle, lastWheelPositions, Pose2d.kZero);
  private TimeInterpolatableBuffer<Pose2d> odomPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(2.0);

  private static final Map<Integer, Pose2d> tagPoses2d = new HashMap<>();

  static {
    for (int i = 1; i <= FieldConstants.aprilTagCount; i++) {
      tagPoses2d.put(
          i,
          FieldConstants.apriltagLayout.getTagPose(i).map(Pose3d::toPose2d).orElse(new Pose2d()));
    }
  }

  private final Map<Integer, TxTyPoseRecord> txTyPoses = new HashMap<>();

  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  private Pose2d trajectorySetpoint = Pose2d.kZero;

  private int lastPriorityTag = -1;

  private static RobotState mInstance;

  private boolean enableSimTrajPoseEstimation = true;

  public static RobotState getInstance() {
    if (mInstance == null) {
      mInstance = new RobotState();
    }
    return mInstance;
  }

  private RobotState() {
    ElasticUI.putPoseEstimate(() -> getEstimatedPose());
  }

  /** Resets the current odometry pose. */
  public void resetPose(Pose2d pose) {
    poseEstimatorOdom.resetPose(pose);
    poseEstimator.resetPose(pose);
    poseEstimatorTxty.resetPose(pose);
  }

  public void setTrajectorySetpoint(Pose2d setpoint) {
    trajectorySetpoint = setpoint;
  }

  public void addDriveSpeeds(ChassisSpeeds speeds) {
    robotVelocity = speeds;
  }

  public void addOdometryObservation(
      double timestamp, Rotation2d yaw, SwerveModulePosition[] wheelPositions) {
    // Update gyro angle
    if (yaw == null) {
      // Derive from kinematics
      yaw =
          lastGyroAngle.rotateBy(
              new Rotation2d(kinematics.toTwist2d(lastWheelPositions, wheelPositions).dtheta));
      lastGyroAngle = yaw;
    }

    lastWheelPositions = wheelPositions;

    poseEstimatorOdom.updateWithTime(timestamp, yaw, wheelPositions);
    poseEstimator.updateWithTime(timestamp, yaw, wheelPositions);
    poseEstimatorTxty.updateWithTime(timestamp, yaw, wheelPositions);
    odomPoseBuffer.addSample(timestamp, poseEstimatorOdom.getEstimatedPosition());
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    Pose2d pose =
        new Pose2d(visionRobotPoseMeters.getTranslation(), getEstimatedPose().getRotation());
    poseEstimator.addVisionMeasurement(pose, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Adds a new timestamped vision measurement. */
  public void addTxTyObservation(TagObservation targetObs) {
    // Get rotation at timestamp
    var sample = odomPoseBuffer.getSample(targetObs.timestamp());
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }
    Rotation2d robotRotation =
        poseEstimator
            .getEstimatedPosition()
            .transformBy(new Transform2d(poseEstimatorOdom.getEstimatedPosition(), sample.get()))
            .getRotation();

    Transform3d cameraPose = VisionConstants.configs[targetObs.camera()].robotToCamera;

    // Use 3D distance and tag angles to find robot pose
    Translation2d camToTagTranslation =
        new Pose3d(Translation3d.kZero, new Rotation3d(0.0, 0.0, -targetObs.tx()))
            .transformBy(
                new Transform3d(new Translation3d(targetObs.distance(), 0, 0), Rotation3d.kZero))
            .getTranslation()
            .rotateBy(new Rotation3d(0, cameraPose.getRotation().getY(), 0))
            .toTranslation2d();
    if (Math.abs(camToTagTranslation.getNorm()) <= 0.01) {
      return;
    }
    Rotation2d camToTagRotation =
        robotRotation.plus(
            cameraPose.getRotation().toRotation2d().plus(camToTagTranslation.getAngle()));
    var tagPose2d = tagPoses2d.get(targetObs.tagId());
    if (tagPose2d == null) return;
    Translation2d fieldToCameraTranslation =
        new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
            .transformBy(new Transform2d(camToTagTranslation.getNorm(), 0.0, new Rotation2d()))
            .getTranslation();
    Pose2d robotPose =
        new Pose2d(
                fieldToCameraTranslation,
                robotRotation.plus(cameraPose.getRotation().toRotation2d()))
            .transformBy(
                new Transform2d(
                    new Pose2d(
                        cameraPose.getTranslation().toTranslation2d(),
                        cameraPose.getRotation().toRotation2d()),
                    Pose2d.kZero));
    // Use gyro angle at time for robot rotation
    robotPose = new Pose2d(robotPose.getTranslation(), robotRotation);

    poseEstimatorTxty.resetTranslation(robotPose.getTranslation());

    if (!txTyPoses.containsKey(targetObs.tagId())
        || targetObs.timestamp() > txTyPoses.get(targetObs.tagId()).timestamp()) {
      // Add transform to current odometry based pose for latency correction
      txTyPoses.put(
          targetObs.tagId(),
          new TxTyPoseRecord(robotPose, camToTagTranslation.getNorm(), targetObs.timestamp()));
    }
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return useTrajectorySetpoint() ? trajectorySetpoint : poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "RobotState/EstimatedOdomPose")
  public Pose2d getEstimatedOdomPose() {
    return poseEstimatorOdom.getEstimatedPosition();
  }

  @AutoLogOutput(key = "RobotState/EstimatedTxtyPose")
  public Pose2d getEstimatedTxtyPose() {
    return poseEstimatorTxty.getEstimatedPosition();
  }

  public Optional<ReefSide> getSideFromTagId(int id) {
    return switch (id) {
      case 6 -> Optional.of(ReefSide.KL);
      case 7 -> Optional.of(ReefSide.AB);
      case 8 -> Optional.of(ReefSide.CD);
      case 9 -> Optional.of(ReefSide.EF);
      case 10 -> Optional.of(ReefSide.GH);
      case 11 -> Optional.of(ReefSide.IJ);

      case 17 -> Optional.of(ReefSide.CD);
      case 18 -> Optional.of(ReefSide.AB);
      case 19 -> Optional.of(ReefSide.KL);
      case 20 -> Optional.of(ReefSide.IJ);
      case 21 -> Optional.of(ReefSide.GH);
      case 22 -> Optional.of(ReefSide.EF);

      default -> Optional.empty();
    };
  }

  public Optional<Integer> getPriorityReefTag() {
    boolean isRed = AllianceFlipUtil.shouldFlip();

    Optional<Integer> tag = Optional.empty();

    for (int i = isRed ? 6 : 17; i < (isRed ? 12 : 23); i++) {
      var curr = txTyPoses.get(i);

      if (curr == null) {
        continue;
      }

      if (Timer.getTimestamp() - curr.timestamp() > txTyObservationStaleSecs.get()
          || curr.distance > maxTagAutoSelectDistance.getAsDouble()) {
        continue;
      }

      if (tag.map(index -> curr.distance() < txTyPoses.get(index).distance()).orElse(true)) {
        tag = Optional.of(i);
      }
    }

    if (tag.isPresent()) {
      lastPriorityTag = tag.get();
      Logger.recordOutput("Robotstate/PrioTag", lastPriorityTag);
    }

    return tag;
  }

  @AutoLogOutput(key = "Robotstate/LastPrioTag")
  public int getLastPriorityTag() {
    return lastPriorityTag;
  }

  public Optional<Pose2d> getTxTyPose(int tagId) {
    if (!txTyPoses.containsKey(tagId)) {
      return Optional.empty();
    }
    var data = txTyPoses.get(tagId);

    boolean isStale = Timer.getTimestamp() - data.timestamp() >= txTyObservationStaleSecs.get();

    Logger.recordOutput("RobotState/IsTxTyStale", isStale);

    // Check if stale
    if (isStale) {
      return Optional.empty();
    }
    // Get odometry based pose at timestamp
    var sample = odomPoseBuffer.getSample(data.timestamp());

    // Latency compensate
    return sample.map(
        pose2d ->
            data.pose().plus(new Transform2d(pose2d, poseEstimatorOdom.getEstimatedPosition())));
  }

  /**
   * Get estimated pose using txty data given tagId on reef and aligned pose on reef. Used for algae
   * intaking and coral scoring.
   */
  public Pose2d getReefPose(int face, Pose2d finalPose) {
    final boolean isRed = AllianceFlipUtil.shouldFlip();
    var tagPose =
        getTxTyPose(
            switch (face) {
              case 1 -> isRed ? 8 : 17;
              case 2 -> isRed ? 9 : 22;
              case 3 -> isRed ? 10 : 21;
              case 4 -> isRed ? 11 : 20;
              case 5 -> isRed ? 6 : 19;
                // 0
              default -> isRed ? 7 : 18;
            });
    // Use estimated pose if tag pose is not present
    if (tagPose.isEmpty()) return RobotState.getInstance().getEstimatedPose();
    // Use distance from estimated pose to final pose to get t value
    final double t =
        MathUtil.clamp(
            (getEstimatedPose().getTranslation().getDistance(finalPose.getTranslation())
                    - minDistanceTagPoseBlend.get())
                / (maxDistanceTagPoseBlend.get() - minDistanceTagPoseBlend.get()),
            0.0,
            1.0);
    return getEstimatedPose().interpolate(tagPose.get(), 1.0 - t);
  }

  public Pose2d getReefPose() {
    var id = getPriorityReefTag();

    if (id.isEmpty()) {
      return RobotState.getInstance().getEstimatedPose();
    }

    var tagPose = getTxTyPose(id.get());

    if (tagPose.isEmpty()) {
      return RobotState.getInstance().getEstimatedPose();
    }

    return tagPose.get();
  }

  private boolean useTrajectorySetpoint() {
    return enableSimTrajPoseEstimation
        ? false
        : Constants.isSim && DriverStation.isAutonomousEnabled();
  }

  public Pose2d getTrajectorySetpoint() {
    return trajectorySetpoint;
  }

  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getEstimatedPose().getRotation());
  }

  public void update() {
    // Log tx/ty poses
    Pose2d[] tagPoses = new Pose2d[FieldConstants.aprilTagCount + 1];
    for (int i = 0; i < FieldConstants.aprilTagCount + 1; i++) {
      tagPoses[i] = getTxTyPose(i).orElse(Pose2d.kZero);
    }
    Logger.recordOutput("RobotState/TxTyPoses", tagPoses);
    getEstimatedTxtyPose();
  }

  public record TxTyPoseRecord(Pose2d pose, double distance, double timestamp) {}
}
