package frc.robot.climber;

public enum ClimberState {
  STOPPED(98.0, 0.0),

  // TODO: Bump up to 12V once verified it works
  LINEUP_FORWARD(100, 1.0),
  LINEUP_BACKWARD(15, 1.0),
  HANGING(99.0, 1.0);

  public final double angle;
  public final double forwardsVoltage;

  private ClimberState(double angle, double forwardVoltage) {
    this.angle = angle;
    this.forwardsVoltage = forwardVoltage;
  }
}
