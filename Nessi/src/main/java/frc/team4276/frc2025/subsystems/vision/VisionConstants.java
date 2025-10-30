package frc.team4276.frc2025.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.Constants.Mode;
import frc.team4276.util.CameraConfig;

public class VisionConstants {
  private static final boolean forceEnableInstanceLogging = false;
  public static final boolean enableInstanceLogging =
      forceEnableInstanceLogging || Constants.getMode() == Mode.REPLAY;

  public static final Transform3d ov9281RobotToCamera =
      new Transform3d(
          Units.inchesToMeters(11.0),
          Units.inchesToMeters(9.0),
          Units.inchesToMeters(8.0),
          new Rotation3d(
              Math.toRadians(0.0), -1.0 * Math.toRadians(20.0), -1.0 * Math.toRadians(20.0)));

  public static final Transform3d ov2311RobotToCamera =
      new Transform3d(
          Units.inchesToMeters(11.0),
          Units.inchesToMeters(-9.0),
          Units.inchesToMeters(8.0),
          new Rotation3d(Math.toRadians(0.0), -1.0 * Math.toRadians(20.0), Math.toRadians(20.0)));

  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final CameraConfig[] configs =
      new CameraConfig[] {
        new CameraConfig("Arducam_OV9281_USB_Camera", ov9281RobotToCamera, 1.25),
        new CameraConfig("Arducam_OV2311_USB_Camera", ov2311RobotToCamera, 1.0)
      };

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.2;
  public static final double fieldBorderMargin = 0.5;
  public static double maxZError = 0.75;
  public static double maxDist = 2.0;
  public static double maxTip = Units.degreesToRadians(20.0);

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians
}
