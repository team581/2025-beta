package frc.robot.lights;

import edu.wpi.first.wpilibj.util.Color;

public enum LightsState {
  ERROR(Color.kRed, BlinkPattern.BLINK_FAST),
  UNHOMED(Color.kYellow, BlinkPattern.BLINK_SLOW),
  HOMED_NO_TAGS(Color.kYellow, BlinkPattern.BLINK_FAST),
  HOMED_SEES_TAGS(Color.kGreen, BlinkPattern.SOLID),

  BLINK(Color.kWhite, BlinkPattern.BLINK_FAST),

  INTAKING_CORAL(Color.kWhite, BlinkPattern.BLINK_SLOW),
  INTAKING_ALGAE(Color.kTeal, BlinkPattern.BLINK_SLOW),

  IDLE_EMPTY(Color.kBlack, BlinkPattern.SOLID),
  HOLDING_CORAL(Color.kWhite, BlinkPattern.SOLID),
  HOLDING_ALGAE(Color.kTeal, BlinkPattern.SOLID),

  LOLLIPOP_SEES_ALGAE(Color.kTeal, BlinkPattern.BLINK_FAST),
  LOLLIPOP_NO_ALGAE(Color.kBlack, BlinkPattern.SOLID),

  CORAL_HANDOFF(Color.kWhite, BlinkPattern.BLINK_FAST),

  CLIMB_LINEUP(Color.kYellow, BlinkPattern.SOLID),
  CLIMB_HANG(Color.kGreen, BlinkPattern.SOLID),
  CLIMB_STOP(Color.kGreen, BlinkPattern.BLINK_SLOW),

  SCORE_NO_ALIGN_NO_TAGS(Color.kYellow, BlinkPattern.BLINK_SLOW),
  SCORE_NO_ALIGN_TAGS(Color.kGreen, BlinkPattern.BLINK_SLOW),
  SCORE_ALIGN_NO_TAGS(Color.kYellow, BlinkPattern.BLINK_FAST),
  SCORE_ALIGN_TAGS(Color.kGreen, BlinkPattern.BLINK_FAST),

  SCORING_ALGAE(Color.kTeal, BlinkPattern.BLINK_FAST),
  SCORING_CORAL(Color.kWhite, BlinkPattern.BLINK_FAST),

  OTHER(Color.kPurple, BlinkPattern.BLINK_SLOW),

  /**
   * @deprecated Replace placeholder lights with actual light patterns.
   */
  @Deprecated
  PLACEHOLDER(Color.kBlack, BlinkPattern.SOLID);

  public final BlinkPattern pattern;
  public final Color color;

  LightsState(Color color, BlinkPattern pattern) {
    this.pattern = pattern;
    this.color = color;
  }
}
