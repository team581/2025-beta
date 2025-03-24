package frc.robot.auto_align;

public enum ReefAlignState {
  ALL_CAMERAS_DEAD(/* aligned= */ false),

  // Checking based on tags
  NO_TAGS_WRONG_POSITION(/* aligned= */ false),
  NO_TAGS_IN_POSITION(/* aligned= */ true),
  HAS_TAGS_WRONG_POSITION(/* aligned= */ false),
  HAS_TAGS_IN_POSITION(/* aligned= */ true);

  public final boolean aligned;

  private ReefAlignState(boolean aligned) {
    this.aligned = aligned;
  }
}
