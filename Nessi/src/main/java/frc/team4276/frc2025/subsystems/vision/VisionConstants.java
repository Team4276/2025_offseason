package frc.team4276.frc2025.subsystems.vision;

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

  public static final double tagHeight = Units.inchesToMeters(6.5);

  public static final Transform3d ov9281RobotToCameraOld =
      new Transform3d(
          Units.inchesToMeters(11.475659),
          Units.inchesToMeters(7.818997),
          Units.inchesToMeters(7.919400),
          new Rotation3d(
              Math.toRadians(5.2362), -1.0 * Math.toRadians(20.0), -1.0 * Math.toRadians(14.1327)));

  public static final Transform3d ov9281RobotToCameraNew =
      new Transform3d(
          Units.inchesToMeters(10.5),
          Units.inchesToMeters(11.25),
          Units.inchesToMeters(9.0),
          new Rotation3d(
              Math.toRadians(0.0), -1.0 * Math.toRadians(20.0), -1.0 * Math.toRadians(0.0)));

  public static final Transform3d ov2311RobotToCameraOld =
      new Transform3d(
          Units.inchesToMeters(8.875327),
          Units.inchesToMeters(7.816629) * -1.0,
          Units.inchesToMeters(17.274095),
          new Rotation3d(Math.toRadians(3.9671), Math.toRadians(15), Math.toRadians(15.5108)));

  public static final Transform3d ov2311RobotToCameraNew =
      new Transform3d(
          Units.inchesToMeters(10.5),
          Units.inchesToMeters(11.25) * -1.0,
          Units.inchesToMeters(9.5),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(15), Math.toRadians(0.0)));

  public static final CameraConfig[] configs =
      new CameraConfig[] {
        new CameraConfig("Arducam_OV9281_USB_Camera", ov9281RobotToCameraOld, 1.25, 48.0, 800),
        new CameraConfig("Arducam_OV2311_USB_Camera", ov2311RobotToCameraOld, 1.0, 60.0, 1200)
      };
}
