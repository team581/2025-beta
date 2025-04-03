package frc.robot.lights;

import edu.wpi.first.wpilibj.util.Color;

public enum LightsState {
  ERROR(Color.kRed, BlinkPattern.BLINK_FAST),
  UNHOMED(Color.kYellow, BlinkPattern.BLINK_SLOW),
  HEALTHY(Color.kGreen, BlinkPattern.SOLID),

  BLINK(Color.kWhite, BlinkPattern.BLINK_FAST),

  IDLE_EMPTY(Color.kGray, BlinkPattern.SOLID),

  INTAKING_CORAL(Color.kWhite, BlinkPattern.BLINK_SLOW),
  INTAKING_ALGAE(Color.kTeal, BlinkPattern.BLINK_SLOW),

  HOLDING_CORAL(Color.kWhite, BlinkPattern.SOLID),
  HOLDING_ALGAE(Color.kTeal, BlinkPattern.SOLID),

  CORAL_HANDOFF(Color.kWhite, BlinkPattern.BLINK_FAST),

  CLIMB_LINEUP(Color.kYellow, BlinkPattern.SOLID),
  CLIMB_HANG(Color.kGreen, BlinkPattern.SOLID),
  CLIMB_STOP(Color.kGreen, BlinkPattern.BLINK_SLOW),

  SCORE_ALIGN_NOT_READY(Color.kCoral, BlinkPattern.BLINK_FAST),
  SCORE_ALIGN_READY(Color.kGreen, BlinkPattern.SOLID),

  SCORING(Color.kGreen, BlinkPattern.BLINK_FAST),

  OTHER(Color.kOrange, BlinkPattern.BLINK_SLOW),

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
