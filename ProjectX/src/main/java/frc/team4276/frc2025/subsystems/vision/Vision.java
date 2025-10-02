package frc.team4276.frc2025.subsystems.vision;

import static frc.team4276.frc2025.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.field.FieldConstants;
import frc.team4276.frc2025.subsystems.vision.VisionIO.TargetObservation;
import frc.team4276.util.AllianceFlipUtil;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private List<TargetObservation> priorityTags = new LinkedList<>();
  private final double priorityStaleTime = 0.5;
  private final double maxPriorityDist = 1.5;

  private boolean[] camerasEnabled = {true, true, true};

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();
    List<Pose3d> allRobotPosesCanceled = new LinkedList<>();
    Map<Integer, TargetObservation> allTxTyObservations = new HashMap<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();
      List<Pose3d> robotPosesCanceled = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        Pose3d robotPose3d = null;
        boolean useVisionRotation = false;
        if (observation.tagCount() > 1) {
          var fieldToRobot =
              observation.fieldToCam1().plus(configs[cameraIndex].robotToCamera.inverse());
          // Add pose to log
          robotPoses.add(fieldToRobot);
          robotPose3d = fieldToRobot;

          useVisionRotation = true;
        } else if (observation.tagCount() == 1) {
          var fieldToRobot0 =
              observation.fieldToCam1().plus(configs[cameraIndex].robotToCamera.inverse());
          var fieldToRobot1 =
              observation.fieldToCam2().plus(configs[cameraIndex].robotToCamera.inverse());

          // Add pose to log
          robotPoses.add(fieldToRobot0);
          robotPoses.add(fieldToRobot1);

          if (Math.abs(fieldToRobot0.minus(fieldToRobot1).getTranslation().getNorm()) <= 0.0001) {
            // robotPosesRejected.add(fieldToRobot0);
            // continue;
          }

          if (observation.ambiguity() < maxAmbiguity) {
            Rotation2d currentRotation = RobotState.getInstance().getEstimatedPose().getRotation();
            Rotation2d visionRotation0 = fieldToRobot0.toPose2d().getRotation();
            Rotation2d visionRotation1 = fieldToRobot1.toPose2d().getRotation();
            if (Math.abs(currentRotation.minus(visionRotation0).getRadians())
                < Math.abs(currentRotation.minus(visionRotation1).getRadians())) {
              robotPose3d = fieldToRobot0;
            } else {
              robotPose3d = fieldToRobot1;
            }
          }

          if (robotPose3d == null) {
            continue;
          }

          if (observation.avgTagDistance() <= 0.01 && observation.avgTagDistance() >= -0.01) {
            robotPosesRejected.add(robotPose3d);
            continue;
          }

          // Exit if robot pose is off the field
          if (robotPose3d.getX() < -fieldBorderMargin
              || robotPose3d.getX() > FieldConstants.fieldLength + fieldBorderMargin
              || robotPose3d.getY() < -fieldBorderMargin
              || robotPose3d.getY() > FieldConstants.fieldWidth + fieldBorderMargin
              || robotPose3d.getZ() < -maxZError
              || robotPose3d.getZ() > maxZError) {
            robotPosesRejected.add(robotPose3d);
            continue;
          }

          // useVisionRotation = false;
          // Calculate standard deviations
          double stdDevFactor =
              Math.pow(observation.avgTagDistance(), 2.0) / observation.tagCount();
          double linearStdDev =
              linearStdDevBaseline * stdDevFactor * configs[cameraIndex].stdDevFactor;
          double angularStdDev =
              useVisionRotation
                  ? angularStdDevBaseline * stdDevFactor * configs[cameraIndex].stdDevFactor
                  : Double.POSITIVE_INFINITY;

          // Send vision observation
          if (camerasEnabled[cameraIndex]) {
            robotPosesAccepted.add(robotPose3d);

            consumer.accept(
                robotPose3d.toPose2d(),
                observation.timestamp(),
                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
          } else {
            robotPosesCanceled.add(robotPose3d);
          }
        }
      }

      // Send Tx Ty Data
      for (var tagObs : inputs[cameraIndex].targetObservations) {
        if (!allTxTyObservations.containsKey(tagObs.tagId())
            || tagObs.distance() < allTxTyObservations.get(tagObs.tagId()).distance()) {
          allTxTyObservations.put(tagObs.tagId(), tagObs);
        }

        if (priorityTags.size() <= cameraIndex) {
          priorityTags.add(tagObs);

        } else if (tagObs.distance() < priorityTags.get(cameraIndex).distance()
            || Timer.getTimestamp() - priorityTags.get(cameraIndex).timestamp()
                > priorityStaleTime) {
          priorityTags.set(cameraIndex, tagObs);
        }
      }

      if (enableInstanceLogging) {
        // Log camera datadata
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
            tagPoses.toArray(new Pose3d[tagPoses.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
            robotPoses.toArray(new Pose3d[robotPoses.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
            robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
            robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesCanceled",
            robotPosesCanceled.toArray(new Pose3d[robotPosesCanceled.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/Enabled",
            camerasEnabled[cameraIndex]);
      }
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
      allRobotPosesCanceled.addAll(robotPosesCanceled);
    }

    allTxTyObservations.values().stream().forEach(RobotState.getInstance()::addTxTyObservation);

    // Log summary data
    Logger.recordOutput("Vision/Summary/Enabled", camerasEnabled);
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesCanceled",
        allRobotPosesCanceled.toArray(new Pose3d[allRobotPosesCanceled.size()]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public void setEnableCamera(int cameraIndex, boolean enable) {
    if (cameraIndex < camerasEnabled.length) {
      camerasEnabled[cameraIndex] = enable;
    }
  }

  public void setCamerasEnabled(boolean... enabled) {
    for (int i = 0; i < enabled.length; i++) {
      setEnableCamera(i, enabled[i]);
    }
  }

  public Command setEnableCameraCommand(int cameraIndex, boolean enabled) {
    return Commands.runOnce(() -> setEnableCamera(cameraIndex, enabled));
  }

  public Command setCamerasEnabledCommand(boolean... enabled) {
    return Commands.runOnce(() -> setCamerasEnabled(enabled));
  }

  public List<TargetObservation> getPriorityTargObs() {
    List<TargetObservation> output = new LinkedList<>();

    for (TargetObservation obs : priorityTags) {
      if (Timer.getTimestamp() - obs.timestamp() < priorityStaleTime
          && obs.distance() < maxPriorityDist
          && (obs.tagId() >= (AllianceFlipUtil.shouldFlip() ? 6 : 17)
              || obs.tagId() <= (AllianceFlipUtil.shouldFlip() ? 11 : 22))) {
        output.add(obs);
      }
    }

    return output;
  }
}
