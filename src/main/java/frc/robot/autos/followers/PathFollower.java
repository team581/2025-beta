package frc.robot.autos.followers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autos.constraints.AutoConstraintOptions;

/** Generates swerve setpoints using robot pose and target pose. */
public interface PathFollower {
  /**
   * Given the current pose of the robot and the target pose, calculate the velocity the robot
   * should drive at to get there.
   *
   * @param currentPose The current pose of the robot.
   * @param currentSpeeds The current speeds of the robot.
   * @param targetPose The target pose to drive to.
   * @return The desired robot velocity to drive at.
   */
  public ChassisSpeeds calculateSpeeds(
      Pose2d currentPose, ChassisSpeeds currentSpeeds, Pose2d targetPose);

  /**
   * Update the constraints for the current path segment.
   *
   * @param constraints The latest constraints for this path segment.
   */
  public default void setConstraints(AutoConstraintOptions constraints) {}

  /**
   * Whether this path follower handles constraints internally, or if they should be calculated
   * externally.
   */
  public default boolean handlesConstraints() {
    return false;
  }
}
