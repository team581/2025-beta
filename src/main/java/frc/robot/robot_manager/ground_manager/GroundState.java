package frc.robot.robot_manager.ground_manager;

public enum GroundState {
  IDLE_EMPTY,
  IDLE_CORAL,

  INTAKING,

  HANDOFF_WAIT,
  HANDOFF_RELEASE,

  OUTWARD_EMPTY,
  OUTWARD_CORAL,

  L1_WAIT,
  L1_SCORE,

  REHOME_DEPLOY,
  UNJAM;
}
