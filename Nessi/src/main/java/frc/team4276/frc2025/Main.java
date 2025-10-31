// Copyright 2021-2024 FRC 6328
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

package frc.team4276.frc2025;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what
 * you are doing, do not modify this file except to change the parameter class
 * to the startRobot
 * call.
 */
public final class Main {
  private Main() {
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>
   * If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    double targetPitch = Units.degreesToRadians(10.0);
    double targetYaw = Units.degreesToRadians(45.0);
    double cameraPitch = Units.degreesToRadians(5.0);
    double cameraYaw = -Units.degreesToRadians(0.0);
    double rawDistToTag = 1;
    Rotation2d robotRotation = Rotation2d.fromDegrees(0.0);
    Pose2d tagPose2d = Pose2d.kZero;
    Pose2d cameraPose = new Pose2d(0.0, 0.0, new Rotation2d(cameraYaw));

    for (int i = 0; i < 1; i++) {
      double pitchVariance = Units.degreesToRadians(0.0);
      double yawVariance = Units.degreesToRadians(0.0);
      double cameraPitchError = Units.degreesToRadians(0.0);
      double cameraYawError = Units.degreesToRadians(0.0);
      double tagDistVariance = 0.0;
      Rotation2d robotRotationError = Rotation2d.fromDegrees(0.0);

      var mechA = mechACalc(
          targetPitch + pitchVariance,
          targetYaw + yawVariance,
          cameraPitch + cameraPitchError,
          cameraYaw + cameraYawError,
          rawDistToTag + tagDistVariance,
          robotRotation.plus(robotRotationError),
          tagPose2d,
          cameraPose);

      var mechB = calculateRobotPose(
          targetPitch + pitchVariance,
          targetYaw + yawVariance,
          cameraPitch + cameraPitchError,
          cameraYaw + cameraYawError,
          rawDistToTag + tagDistVariance,
          robotRotation.plus(robotRotationError),
          tagPose2d,
          cameraPose);
        
      var pv = PhotonUtils.estimateFieldToRobot(0.0, targetYaw, cameraPitchError, targetPitch, robotRotation, robotRotationError, cameraPose, null)

      System.out.println("Mech A:");
      System.out.println("x: " + mechA.getX());
      System.out.println("y: " + mechA.getY());
      System.out.println("rot: " + mechA.getRotation().getDegrees());
      System.out.println("dist: " + mechA.getTranslation().getNorm());

      System.out.println("Mech B:");
      System.out.println("x: " + mechB.getX());
      System.out.println("y: " + mechB.getY());
      System.out.println("rot: " + mechB.getRotation().getDegrees());
      System.out.println("dist: " + mechB.getTranslation().getNorm());

      System.out.println("Variance: " + mechA.getTranslation().getDistance(mechB.getTranslation()));
      System.out.println(
          "Variance A: " + mechA.getTranslation().getDistance(new Translation2d(-1.0, 0.0)));
      System.out.println(
          "Variance B: " + mechB.getTranslation().getDistance(new Translation2d(-1.0, 0.0)));
    }

    // RobotBase.startRobot(Robot::new);
  }

  private static Pose2d mechACalc(
      double targetPitch,
      double targetYaw,
      double cameraPitch,
      double cameraYaw,
      double rawDistToTag,
      Rotation2d robotRotation,
      Pose2d tagPose2d,
      Pose2d cameraPose) {
    // Use 3D distance and tag angles to find robot pose
    Translation2d camToTagTranslation = new Pose3d(Translation3d.kZero, new Rotation3d(0, targetPitch, -targetYaw))
        .transformBy(new Transform3d(new Translation3d(rawDistToTag, 0, 0), Rotation3d.kZero))
        .getTranslation()
        .rotateBy(new Rotation3d(0.0, cameraPitch, 0.0))
        .toTranslation2d();
    Rotation2d camToTagRotation = robotRotation.plus(camToTagTranslation.getAngle().plus(new Rotation2d(cameraYaw)));
    Translation2d fieldToCameraTranslation = new Pose2d(tagPose2d.getTranslation(),
        camToTagRotation.plus(Rotation2d.kPi))
        .transformBy(new Transform2d(camToTagTranslation.getNorm(), 0.0, Rotation2d.kZero))
        .getTranslation();
    Pose2d robotPose = new Pose2d(fieldToCameraTranslation, robotRotation.plus(new Rotation2d(cameraYaw)))
        .transformBy(new Transform2d(cameraPose, Pose2d.kZero));
    // Use gyro angle at time for robot rotation
    return new Pose2d(robotPose.getTranslation(), robotRotation);
  }

  private static Pose2d calculateRobotPose(
      double targetPitch,
      double targetYaw,
      double cameraPitch,
      double cameraYaw,
      double rawDistToTag,
      Rotation2d robotRotation,
      Pose2d tagPose2d,
      Pose2d cameraPose) {
    // double distanceToTagMechA = new Rotation2d(targetPitch).plus(new Rotation2d(cameraPitch)).getCos() * rawDistToTag;
    Translation2d translationbruh = new Pose3d(Translation3d.kZero, new Rotation3d(0, targetPitch, -targetYaw))
    .transformBy(new Transform3d(new Translation3d(rawDistToTag, 0, 0), Rotation3d.kZero))
    .getTranslation()
    .rotateBy(new Rotation3d(0.0, cameraPitch, 0.0))
    .toTranslation2d();

    double distanceToTagMechA = translationbruh.getNorm();

    var cameraToRobotCenter = cameraPose.getTranslation().unaryMinus().rotateBy(robotRotation);

    var angleToTag = new Rotation2d(translationbruh.getAngle().getRadians()).plus(robotRotation.plus(new Rotation2d(cameraYaw)));

    var translation = new Translation2d(distanceToTagMechA, angleToTag);

    var translatedPose = tagPose2d.getTranslation().minus(translation);

    var fieldRelativeRobotTranslation = translatedPose.plus(cameraToRobotCenter);

    return new Pose2d(fieldRelativeRobotTranslation, robotRotation);
  }
}
