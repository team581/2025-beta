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
      FeatureFlag.of("Vision/StaleDataRejection", false);

  public static final BooleanSupplier MT_VISION_METHOD =
      FeatureFlag.of("Vision/MTVisionMethod", true);

  public static final BooleanSupplier FIELD_CALIBRATION = FeatureFlag.of("FieldCalibration", false);

  public static final BooleanSupplier COLLISION_AVOIDANCE_OBSTRUCTION =
      FeatureFlag.of("CollisionAvoidance/Obstruction", true);

  public static final BooleanSupplier USE_ALTERNATE_WAYPOINT_CHOOSER =
      FeatureFlag.of("CollisionAvoidance/AlternateClosestWaypointChooser", true);

  public static final BooleanSupplier APPROACH_TAG_CHECK =
      FeatureFlag.of("Vision/ApproachTagCheck", false);

  public static final BooleanSupplier VISION_HANDOFF_ADJUSTMENT =
      FeatureFlag.of("Vision/HandoffAdjustment", true);

  public static final BooleanSupplier AUTO_ALIGN_AUTO_SCORE =
      FeatureFlag.of("AutoAlign/AutoScore", false);

  public static final BooleanSupplier SPIN_TO_WIN = FeatureFlag.of("Yapping/SpinToWin", false);

  private FeatureFlags() {}
}
