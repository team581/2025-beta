package frc.robot.vision;

public enum VisionState {
  /** Any AprilTags. */
  TAGS,
  /** The AprilTag on the side of the reef we are scoring on. */
  CLOSEST_REEF_TAG,
  HANDOFF,
  CORAL_DETECTION,
  ALGAE_DETECTION;
}
