package frc.robot.lights;

public enum BlinkPattern {
  SOLID(0.0),
  BLINK_FAST(0.08),
  BLINK_SLOW(0.25);

  public final double duration;

  private BlinkPattern(double duration) {
    this.duration = duration;
  }
}
