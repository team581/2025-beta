package frc.robot.robot_manager.ground_manager;

public enum GroundState {
  IDLE_EMPTY,
  IDLE_CORAL,

  INTAKING,

  HANDOFF_WAIT,
  HANDOFF_RELEASE,

  L1_WAIT,
  L1_SCORE,

  UNOBSTRUCT_ARM_NO_CORAL,
  UNOBSTRUCT_ARM_W_CORAL,

  REHOME_DEPLOY,
  UNJAM;
}
