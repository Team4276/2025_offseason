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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    double rawDistToTag = 2.0;
    double pitch = 30.0;
    double a = 1;

    for (int i = 0; i < 20; i++) {
      System.out.println(
          calc3dDist(rawDistToTag + (i * 0.005), a)
              - calcMechADist(pitch, rawDistToTag + (i * 0.005)));
    }

    System.out.println("pitch error");

    for (int i = 0; i < 20; i++) {
      System.out.println(
          calc3dDist(rawDistToTag, a) - calcMechADist(pitch + (i * 0.01), rawDistToTag));
    }

    double distanceToTagMeters = 1;
    double cameraY = 1.0;
    Rotation2d cameraYaw = Rotation2d.fromDegrees(-30);
    Rotation2d angleToTarget = Rotation2d.fromDegrees(60);

    System.out.println(
        calculateRobotPose(
                distanceToTagMeters + 0.1,
                angleToTarget,
                cameraYaw,
                new Translation2d(1.0, cameraY))
            .getTranslation()
            .getDistance(
                calculateRobotPose(
                        distanceToTagMeters - 0.05,
                        angleToTarget.unaryMinus(),
                        cameraYaw.unaryMinus(),
                        new Translation2d(1.0, -cameraY))
                    .getTranslation()));

    // RobotBase.startRobot(Robot::new);
  }

  private static double calc3dDist(double rawDistToTag, double a) {
    return Math.sqrt(rawDistToTag * rawDistToTag - (a * a));
  }

  private static double calcMechADist(double pitch, double rawDistToTag) {
    return Rotation2d.fromDegrees(pitch).getCos() * rawDistToTag;
  }

  private static Pose2d calculateRobotPose(
      double distanceToTagMeters,
      Rotation2d horizontalAngleToTarget,
      Rotation2d cameraYaw,
      Translation2d robotToCamera) {
    var tagPose = Pose2d.kZero;

    var robotRotation = Rotation2d.fromDegrees(30.0);

    var correctedCameraRotation = robotRotation.plus(cameraYaw);

    var scaledTx = Rotation2d.fromDegrees(-horizontalAngleToTarget.div(1.0).getDegrees());

    var cameraToRobotCenter = robotToCamera.unaryMinus().rotateBy(robotRotation);

    var angleToTag = scaledTx.plus(correctedCameraRotation);

    var translation = new Translation2d(distanceToTagMeters, angleToTag);
    var translatedPose = tagPose.getTranslation().minus(translation);

    var fieldRelativeRobotTranslation = translatedPose.plus(cameraToRobotCenter);

    return new Pose2d(fieldRelativeRobotTranslation, robotRotation);
  }
}
