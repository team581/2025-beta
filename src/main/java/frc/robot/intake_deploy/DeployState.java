package frc.robot.intake_deploy;

public enum DeployState {
  UNHOMED(0.0),
  HOMING(0.0), // TODO: set homing end angle
  STOWED(0.0),
  FLOOR_INTAKE(0.0),
  HANDOFF(0.0),
  L1_SCORE(0.0),

  UNJAM(0.0);

  public final double angle;

  private DeployState(double angle) {
    this.angle = angle;
  }
}
