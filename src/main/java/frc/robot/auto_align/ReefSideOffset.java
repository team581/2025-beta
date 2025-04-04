package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig;

public enum ReefSideOffset {
  BASE(
      new Transform2d(0, 0, Rotation2d.fromDegrees(90)),
      new Transform2d(0, 0, Rotation2d.fromDegrees(270))),
  ALGAE_INTAKING(
      new Transform2d(
          // Half of drivebase + bumper side width + reef side to bumper distance
          -Units.inchesToMeters(14.5 + 4.0 + 5.0),
          Units.inchesToMeters(RobotConfig.get().arm().inchesFromCenter()),
          Rotation2d.fromDegrees(270)),
      new Transform2d(
          -Units.inchesToMeters(14.5 + 4.0 + 5.0),
          -Units.inchesToMeters(RobotConfig.get().arm().inchesFromCenter()),
          Rotation2d.fromDegrees(90))),
  ALGAE_RAISING(
      new Transform2d(
          -Units.inchesToMeters(14.5 + 4.0 + 20.0),
          Units.inchesToMeters(RobotConfig.get().arm().inchesFromCenter()),
          Rotation2d.fromDegrees(270)),
      new Transform2d(
          -Units.inchesToMeters(14.5 + 4.0 + 20.0),
          -Units.inchesToMeters(RobotConfig.get().arm().inchesFromCenter()),
          Rotation2d.fromDegrees(90)));

  public final Transform2d leftOffset;
  public final Transform2d rightOffset;

  private ReefSideOffset(Transform2d leftOffset, Transform2d rightOffset) {
    this.leftOffset = leftOffset;
    this.rightOffset = rightOffset;
  }
}
