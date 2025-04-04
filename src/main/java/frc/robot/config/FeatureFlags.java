package frc.robot.config;

import frc.robot.util.FeatureFlag;
import java.util.function.BooleanSupplier;

public class FeatureFlags {
  public static final BooleanSupplier PURE_PURSUIT_USE_DYNAMIC_LOOKAHEAD =
      FeatureFlag.of("PurePursuit/UseDynamicLookahead", true);

  public static final BooleanSupplier PURE_PURSUIT_ROTATE_IMMEDIATELY =
      FeatureFlag.of("PurePursuit/RotateImmediately", false);

  public static final BooleanSupplier CAMERA_POSITION_CALIBRATION =
      FeatureFlag.of("Vision/PositionCalibrationMode", false);

  public static final BooleanSupplier CORAL_DETECTION = FeatureFlag.of("CoralMap", false);

  public static final BooleanSupplier VISION_STALE_DATA_CHECK =
      FeatureFlag.of("Vision/StaleDataRejection", true);

  public static final BooleanSupplier CONTEXT_BASED_MEGATAG_1 =
      FeatureFlag.of("Vision/ContextBasedMegatag1", false);

  public static final BooleanSupplier FIELD_CALIBRATION = FeatureFlag.of("FieldCalibration", false);

  public static final BooleanSupplier COLLISION_AVOIDANCE_OBSTRUCTION =
      FeatureFlag.of("CollisionAvoidance/Obstruction", true);

  public static final BooleanSupplier VISION_HANDOFF_ADJUSTMENT =
      FeatureFlag.of("Vision/HandoffAdjustment", true);

  public static final BooleanSupplier AUTO_ALIGN_AUTO_SCORE =
      FeatureFlag.of("AutoAlign/AutoScore", false);

  /**
   * Whether L4 approach state should automatically transition to L4 lineup when close enough to the
   * reef during autonomous. Doesn't alter teleop behavior.
   */
  public static final BooleanSupplier EXPLICIT_L4_LINEUP =
      FeatureFlag.of("Auto/ExplicitL4Lineup", true);

  public static final BooleanSupplier SPIN_TO_WIN = FeatureFlag.of("Yapping/SpinToWin", false);

  private FeatureFlags() {}
}
