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

  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final CameraConfig[] configs =
      new CameraConfig[] {
        new CameraConfig(
            "Arducam_OV9281_USB_Camera",
            new Transform3d(
                Units.inchesToMeters(11.475659),
                Units.inchesToMeters(7.818997),
                Units.inchesToMeters(7.919400),
                new Rotation3d(
                    Math.toRadians(5.2362),
                    -1.0 * Math.toRadians(20.0),
                    -1.0 * Math.toRadians(14.1327))),
            1.25),
        // new CameraConfig(
        // "USB_Camera",
        // new Transform3d(
        // Units.inchesToMeters(8.822182),
        // Units.inchesToMeters(7.831371) * -1.0,
        // Units.inchesToMeters(17.288335),
        // new Rotation3d(
        // Math.toRadians(3.9671) - (Math.PI / 2),
        // Math.toRadians(15),
        // Math.toRadians(15.5108))),
        // 1.5),
        new CameraConfig(
            "Arducam_OV2311_USB_Camera",
            new Transform3d(
                Units.inchesToMeters(8.875327),
                Units.inchesToMeters(7.816629) * -1.0,
                Units.inchesToMeters(17.274095),
                new Rotation3d(
                    Math.toRadians(3.9671), Math.toRadians(15), Math.toRadians(15.5108))),
            1.0),
        new CameraConfig(
            "Camera_Module_v1",
            new Transform3d(
                Units.inchesToMeters(-1.394869),
                Units.inchesToMeters(8.628504) * -1.0,
                Units.inchesToMeters(36.999197),
                new Rotation3d(0.0, -1.0 * Math.toRadians(35.0), Math.toRadians(165))),
            1.5)
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
