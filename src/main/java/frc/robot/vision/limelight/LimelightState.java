package frc.robot.vision.limelight;

public enum LimelightState {
  OFF(0),
  TAGS(1),
  CLOSEST_REEF_TAG(1),
  CLOSEST_REEF_TAG_CLOSEUP(1),
  STATION_TAGS(1),
  CORAL(2),
  ALGAE(4),
  HELD_CORAL(5);

  final int pipelineIndex;

  LimelightState(int index) {
    this.pipelineIndex = index;
  }
}
