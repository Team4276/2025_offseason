package frc.team4276.lib;

import edu.wpi.first.math.geometry.Transform3d;

public class CameraConfig {
  public final String name;
  public final Transform3d robotToCamera;

  public CameraConfig(String name, Transform3d robotToCamera) {
    this.name = name;
    this.robotToCamera = robotToCamera;
  }
}
