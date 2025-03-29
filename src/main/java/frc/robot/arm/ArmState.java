package frc.robot.arm;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum ArmState {
  /**
   * @deprecated This is a placeholder state.
   */
  @Deprecated
  UNTUNED(90),

  PRE_MATCH_HOMING(0),

  UNJAM(UNTUNED),
  // Idle positions
  HOLDING_UPRIGHT(90),

  // Left coral positions
  CORAL_SCORE_LEFT_LINEUP_L2(180 - 43.0),
  CORAL_SCORE_LEFT_RELEASE_L2(180 - 15.5),

  CORAL_SCORE_LEFT_LINEUP_L3(180 - 43.0),
  CORAL_SCORE_LEFT_RELEASE_L3(180 - 15.0),

  CORAL_SCORE_LEFT_LINEUP_L4(180 - 47.5),
  CORAL_SCORE_LEFT_RELEASE_L4(180 + 10.0),

  // Right coral positions
  CORAL_SCORE_RIGHT_LINEUP_L1(35.0),
  CORAL_SCORE_RIGHT_RELEASE_L1(35.0),

  CORAL_SCORE_RIGHT_LINEUP_L2(43.0),
  CORAL_SCORE_RIGHT_RELEASE_L2(15.5),

  CORAL_SCORE_RIGHT_LINEUP_L3(43.0),
  CORAL_SCORE_RIGHT_RELEASE_L3(15.0),

  CORAL_SCORE_RIGHT_LINEUP_L4(47.5),
  CORAL_SCORE_RIGHT_RELEASE_L4(-10.0),

  // Handoffs
  CORAL_HANDOFF(-90.0),

  // Algae positions
  ALGAE_INTAKE_FLOOR(-21.18),

  ALGAE_INTAKE_LEFT_L2(180.0),
  ALGAE_INTAKE_LEFT_L3(180.0),

  ALGAE_INTAKE_RIGHT_L2(0.0),
  ALGAE_INTAKE_RIGHT_L3(0.0),

  ALGAE_NET_LEFT(UNTUNED),

  ALGAE_NET_RIGHT(UNTUNED),

  ALGAE_PROCESSOR(UNTUNED),

  ALGAE_OUTTAKE(UNTUNED),

  COLLISION_AVOIDANCE(UNTUNED),
  CLIMBING(UNTUNED),

  SPIN_TO_WIN(90),

  // For auto
  LOLLIPOP_CORAL_INTAKE_APPROACH(20),
  LOLLIPOP_CORAL_INTAKE_INTAKE(-8),
  LOLLIPOP_CORAL_INTAKE_TILT(-15);

  private final double defaultAngle;
  private final DoubleSubscriber tunableAngle;

  ArmState(double angle) {
    this.defaultAngle = angle;
    this.tunableAngle = DogLog.tunable("Arm/State/" + name(), angle);
  }

  ArmState(ArmState other) {
    this(other.defaultAngle);
  }

  public double getAngle() {
    return tunableAngle.get();
  }
}
