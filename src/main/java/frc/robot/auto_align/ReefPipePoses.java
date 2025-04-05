package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;

// Compute the offset poses for scoring on the reef once on boot to reduce GC pressure from dynamic
// computation
record ReefPipePoses(
    Pose2d base,
    Pose2d leftL1,
    Pose2d leftL2,
    Pose2d leftL3,
    Pose2d leftL4,
    Pose2d leftRaising,
    Pose2d leftBackaway,
    Pose2d rightL1,
    Pose2d rightL2,
    Pose2d rightL3,
    Pose2d rightL4,
    Pose2d rightRaising,
    Pose2d rightBackaway) {
  public ReefPipePoses(Pose2d base) {
    this(
        base,
        base.transformBy(ReefPipeLevel.L1.leftOffset),
        base.transformBy(ReefPipeLevel.L2.leftOffset),
        base.transformBy(ReefPipeLevel.L3.leftOffset),
        base.transformBy(ReefPipeLevel.L4.leftOffset),
        base.transformBy(ReefPipeLevel.RAISING.leftOffset),
        base.transformBy(ReefPipeLevel.BACK_AWAY.leftOffset),
        base.transformBy(ReefPipeLevel.L1.rightOffset),
        base.transformBy(ReefPipeLevel.L2.rightOffset),
        base.transformBy(ReefPipeLevel.L3.rightOffset),
        base.transformBy(ReefPipeLevel.L4.rightOffset),
        base.transformBy(ReefPipeLevel.RAISING.rightOffset),
        base.transformBy(ReefPipeLevel.BACK_AWAY.rightOffset));
  }

  public Pose2d getLeftPose(ReefPipeLevel level) {
    return switch (level) {
      case BASE -> base;
      case L1 -> leftL1;
      case L2 -> leftL2;
      case L3 -> leftL3;
      case L4 -> leftL4;
      case RAISING -> leftRaising;
      case BACK_AWAY -> leftBackaway;
    };
  }

  public Pose2d getRightPose(ReefPipeLevel level) {
    return switch (level) {
      case BASE -> base;
      case L1 -> rightL1;
      case L2 -> rightL2;
      case L3 -> rightL3;
      case L4 -> rightL4;
      case RAISING -> rightRaising;
      case BACK_AWAY -> rightBackaway;
    };
  }
}
