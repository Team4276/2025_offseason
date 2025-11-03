package frc.team4276.frc2025.subsystems.vision;

import static frc.team4276.frc2025.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.field.FieldConstants;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private final List<Pose3d> allTagPoses = new LinkedList<>();
  private final List<Pose3d> allTxTyPoses = new LinkedList<>();
  private final List<Pose3d> allRobotPoses = new LinkedList<>();

  public Vision(VisionIO... io) {
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
    allTagPoses.clear();
    allTxTyPoses.clear();
    allRobotPoses.clear();

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      io[cameraIndex].updateInputs(inputs[cameraIndex]);
      Logger.processInputs("Vision/Camera_" + Integer.toString(cameraIndex), inputs[cameraIndex]);

      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> txtyPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = FieldConstants.apriltagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Send Tx Ty Data
      for (var tagObs : inputs[cameraIndex].targetObservations) {
        txtyPoses.add(new Pose3d(tagObs.robotPose()));
      }

      RobotState.getInstance().addVisionObservation(inputs[cameraIndex].targetObservations);

      for (var poseObs : inputs[cameraIndex].poseObservations) {
        robotPoses.add(poseObs.robotPose());
      }

      RobotState.getInstance().addVision3dPoseObservation(inputs[cameraIndex].poseObservations);

      if (enableInstanceLogging) {
        // Log camera datadata
        Logger.recordOutput(
            "Vision/Camera_" + Integer.toString(cameraIndex) + "/TagPoses",
            tagPoses.toArray(new Pose3d[tagPoses.size()]));
        Logger.recordOutput(
            "Vision/Camera_" + Integer.toString(cameraIndex) + "/TxtyPoses",
            txtyPoses.toArray(new Pose3d[txtyPoses.size()]));
        Logger.recordOutput(
            "Vision/Camera_" + Integer.toString(cameraIndex) + "/RobotPoses",
            robotPoses.toArray(new Pose3d[robotPoses.size()]));
      }
      allTagPoses.addAll(tagPoses);
      allTxTyPoses.addAll(txtyPoses);
      allRobotPoses.addAll(robotPoses);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/TxtyPoses", allTxTyPoses.toArray(new Pose3d[allTxTyPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
  }
}
