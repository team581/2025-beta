package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig;

public enum ReefPipeLevel {
  BASE(
      new Transform2d(0, 0, Rotation2d.fromDegrees(90)),
      new Transform2d(0, 0, Rotation2d.fromDegrees(270))),
  L1(
      new Transform2d(
          // Half of drivebase + bumper side width + 1 coral width + reef side to pipe distance +
          // extra
          -Units.inchesToMeters(14.5 + 4.0 + 4.5 + 2.0 + 2.0),
          Units.inchesToMeters(RobotConfig.get().arm().inchesFromCenter()),
          Rotation2d.fromDegrees(270)),
      new Transform2d(
          -Units.inchesToMeters(14.5 + 4.0 + 4.5 + 2.0 + 2.0),
          -Units.inchesToMeters(RobotConfig.get().arm().inchesFromCenter()),
          Rotation2d.fromDegrees(90))),
  L2(
      new Transform2d(
          -Units.inchesToMeters(14.5 + 4.0 + 4.5 + 2.0 + 2.0),
          Units.inchesToMeters(RobotConfig.get().arm().inchesFromCenter()),
          Rotation2d.fromDegrees(270)),
      new Transform2d(
          -Units.inchesToMeters(14.5 + 4.0 + 4.5 + 2.0 + 2.0),
          -Units.inchesToMeters(RobotConfig.get().arm().inchesFromCenter()),
          Rotation2d.fromDegrees(90))),
  L3(
      new Transform2d(
          -Units.inchesToMeters(14.5 + 4.0 + 4.5 + 2.0 + 2.0),
          Units.inchesToMeters(RobotConfig.get().arm().inchesFromCenter()),
          Rotation2d.fromDegrees(270)),
      new Transform2d(
          -Units.inchesToMeters(14.5 + 4.0 + 4.5 + 2.0 + 2.0),
          -Units.inchesToMeters(RobotConfig.get().arm().inchesFromCenter()),
          Rotation2d.fromDegrees(90))),
  L4(
      new Transform2d(
          -Units.inchesToMeters(14.5 + 4.0 + 4.5 + 2.0 + 2.0),
          Units.inchesToMeters(RobotConfig.get().arm().inchesFromCenter()),
          Rotation2d.fromDegrees(270)),
      new Transform2d(
          -Units.inchesToMeters(14.5 + 4.0 + 4.5 + 2.0 + 2.0),
          -Units.inchesToMeters(RobotConfig.get().arm().inchesFromCenter()),
          Rotation2d.fromDegrees(90))),

  RAISING(
      new Transform2d(
          -Units.inchesToMeters(14.5 + 4.0 + 4.5 + 20.0),
          Units.inchesToMeters(RobotConfig.get().arm().inchesFromCenter()),
          Rotation2d.fromDegrees(270)),
      new Transform2d(
          -Units.inchesToMeters(14.5 + 4.0 + 4.5 + 20.0),
          -Units.inchesToMeters(RobotConfig.get().arm().inchesFromCenter()),
          Rotation2d.fromDegrees(90)));

  public final Transform2d leftOffset;
  public final Transform2d rightOffset;

  private ReefPipeLevel(Transform2d leftOffset, Transform2d rightOffset) {
    this.leftOffset = leftOffset;
    this.rightOffset = rightOffset;
  }
}
