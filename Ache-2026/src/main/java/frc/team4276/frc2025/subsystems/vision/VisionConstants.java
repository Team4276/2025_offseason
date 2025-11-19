// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.team4276.frc2025.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.Constants.Mode;
import frc.team4276.lib.CameraConfig;

import java.util.ArrayList;
import java.util.List;

public class VisionConstants {
  private static final boolean forceEnableInstanceLogging = false;
  public static final boolean enableInstanceLogging =
      forceEnableInstanceLogging || Constants.getMode() == Mode.REPLAY;

  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Robot to camera transforms
  public static final Transform3d ov9281RobotToCamera = new Transform3d(
      Units.inchesToMeters(11.0),
      Units.inchesToMeters(9.0),
      Units.inchesToMeters(8.0),
      new Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(-20.0)));

  public static final Transform3d ov2311RobotToCamera = new Transform3d(
      Units.inchesToMeters(11.0),
      Units.inchesToMeters(9.0) * -1.0,
      Units.inchesToMeters(8.0),
      new Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(20.0)));

  public static final CameraConfig[] configs = new CameraConfig[] {
      new CameraConfig("Arducam_OV9281_USB_Camera", ov9281RobotToCamera),
      new CameraConfig("Arducam_OV2311_USB_Camera", ov2311RobotToCamera)
  };

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.05;
  public static double maxZError = 0.75;
  public static double maxSingleTagDistanceMeters = 3.0;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Vision can sometimes provide bad rotation updates
  public static boolean useVisionRotation = true;
  public static boolean useVisionRotationSingleTag = false;

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors = new double[] {
      1.0, // Camera 0
      1.0 // Camera 1
  };

  public static final List<Integer> singleTagIdsToReject = new ArrayList<>() {
    {
      // Red Side
      add(1); // Feeder Station
      add(2); // Feeder Station
      add(3); // Processor
      add(4); // Blue Barge
      add(5); // Red Barge

      // Blue Side
      add(12); // Feeder Station
      add(13); // Feeder Station
      add(14); // Blue Barge
      add(15); // Red Barge
      add(16); // Red Processor
    }
  };
}