package frc.robot.vision.game_piece_detection;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.tag_align.AlignmentCostUtil;
import frc.robot.config.FeatureFlags;
import frc.robot.fms.FmsSubsystem;
import frc.robot.intake_assist.IntakeAssistUtil;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.limelight.LimelightHelpers;
import frc.robot.vision.results.GamePieceResult;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public class CoralMap extends StateMachine<CoralMapState> {
  private static final int LOLLIPOP_FILTER_TAPS = 7;
  private static final double SWERVE_MAX_LINEAR_SPEED_TRACKING = 3.0;
  private static final double SWERVE_MAX_ANGULAR_SPEED_TRACKING = 3.0;

  private static final double CORAL_LIFETIME_SECONDS = 1.5;

  private static final double LOLLIPOP_LIFTEIME_SECONDS = 2.0;

  // TODO: UPDATE THESE TO REAL NUMBERS
  private static final double CAMERA_IMAGE_HEIGHT = 240.0;
  private static final double CAMERA_IMAGE_WIDTH = 320.0;
  private static final double FOV_VERTICAL = 48.9;
  private static final double FOV_HORIZONTAL = 62.5;
  private static final double HORIZONTAL_LEFT_VIEW = 62.5 / 2;
  private static final double VERTICAL_TOP_VIEW = 48.9 / 2;

  private static final String LIMELIGHT_NAME = "limelight-coral";
  private static final NetworkTableEntry LL_TCORNXY =
      NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("tcornxy");

  private final ArrayList<CoralMapElement> coralMap = new ArrayList<>();
  private double[] previousCornersArray = new double[0];
  private boolean staleCoralCorners = false;
  private ChassisSpeeds swerveSpeeds = new ChassisSpeeds();
  private LocalizationSubsystem localization;
  private SwerveSubsystem swerve;

  private final LinearFilter lollipopXFilter = LinearFilter.movingAverage(LOLLIPOP_FILTER_TAPS);
  private final LinearFilter lollipopYFilter = LinearFilter.movingAverage(LOLLIPOP_FILTER_TAPS);
  private double filteredLollipopX = 0.0;
  private double filteredLollipopY = 0.0;

  private final Comparator<Pose2d> bestCoralComparator =
      Comparator.comparingDouble(
          target ->
              AlignmentCostUtil.getCoralAlignCost(
                  target, localization.getPose(), swerve.getFieldRelativeSpeeds()));

  private Optional<Pose2d> filteredLollipopPose = Optional.empty();
  private double lastLollipopTime = 0.0;

  public CoralMap(LocalizationSubsystem localization, SwerveSubsystem swerve) {
    super(SubsystemPriority.VISION, CoralMapState.DEFAULT_STATE);
    this.localization = localization;
    this.swerve = swerve;
  }

  private void resetLollipopFilter(Translation2d expectedTranslation) {
    for (var i = 0; i < LOLLIPOP_FILTER_TAPS; i++) {
      lollipopXFilter.calculate(expectedTranslation.getX());
      lollipopYFilter.calculate(expectedTranslation.getY());
    }
  }

  public void updateLollipopResult(Optional<GamePieceResult> lollipopResult) {
    var newPose =
        IntakeAssistUtil.getLollipopIntakePoseFromVisionResult(
            lollipopResult, localization.getPose());
    if (newPose.isPresent()
        && safeToTrack()
        && isLollipopInSafeSpotForAuto(newPose.get().getTranslation())) {
      if (filteredLollipopPose.isEmpty()) {
        resetLollipopFilter(newPose.get().getTranslation());
      }
      filteredLollipopX = lollipopXFilter.calculate(newPose.get().getX());
      filteredLollipopY = lollipopYFilter.calculate(newPose.get().getY());
      filteredLollipopPose =
          Optional.of(
              new Pose2d(
                  filteredLollipopX,
                  filteredLollipopY,
                  Rotation2d.fromDegrees(newPose.get().getRotation().getDegrees())));
      lastLollipopTime = Timer.getFPGATimestamp();
    } else {
      if (Timer.getFPGATimestamp() - lastLollipopTime >= LOLLIPOP_LIFTEIME_SECONDS) {
        filteredLollipopPose = Optional.empty();
        lollipopXFilter.reset();
        lollipopYFilter.reset();
      }
    }
  }

  public Optional<Pose2d> getLollipopIntakePose() {
    return filteredLollipopPose;
  }

  private List<Translation2d> getRawCoralPoses() {
    List<Translation2d> coralTranslations = new ArrayList<>();
    double[] corners = LL_TCORNXY.getDoubleArray(new double[0]);

    // Check if the result array has changed
    staleCoralCorners = Arrays.equals(previousCornersArray, corners);
    previousCornersArray = corners;

    if (staleCoralCorners) {
      DogLog.timestamp("CoralMap/SkipStaleCorners");

      return List.of();
    }

    double latency =
        (LimelightHelpers.getLatency_Capture(LIMELIGHT_NAME)
                + LimelightHelpers.getLatency_Pipeline(LIMELIGHT_NAME))
            / 1000.0;
    double timestamp = Timer.getFPGATimestamp() - latency;
    var robotPoseAtCapture = localization.getPose(timestamp);

    // Loop through 4 points
    // Delete 3 point data
    if (corners.length >= 8 && corners[0] != 0.0 && corners.length % 8 == 0) {

      for (int i = 0; i < corners.length; i = i + 8) {
        var centerX = (corners[i] + corners[i + 2]) / 2.0;
        var centerY = (corners[i + 1] + corners[i + 5]) / 2.0;

        double angleX = (((centerX / CAMERA_IMAGE_WIDTH) * FOV_HORIZONTAL) - HORIZONTAL_LEFT_VIEW);
        double angleY =
            -1.0 * (((centerY / CAMERA_IMAGE_HEIGHT) * FOV_VERTICAL) - VERTICAL_TOP_VIEW);
        var maybeCoralPose =
            GamePieceDetectionUtil.calculateFieldRelativeCoralTranslationFromCamera(
                robotPoseAtCapture, new GamePieceResult(angleX, angleY, 0));

        coralTranslations.add(maybeCoralPose);
      }
    }

    return coralTranslations;
  }

  public Optional<Pose2d> getBestCoral() {
    if (coralMap.isEmpty()) {
      return Optional.empty();
    }

    var bestCoral =
        coralMap.stream()
            .map(coral -> new Pose2d(coral.coralTranslation(), Rotation2d.kZero))
            .min(bestCoralComparator);

    if (bestCoral.isPresent()) {
      DogLog.log("CoralMap/BestCoral", bestCoral.orElseThrow());
    }

    return bestCoral;
  }

  public static boolean isCoralInSafeSpotForAuto(Translation2d coralPose) {
    var centerOfReef = AutoAlign.getAllianceCenterOfReef();
    if (coralPose.getDistance(centerOfReef) < Units.inchesToMeters(37.2)) {
      return false;
    }

    if ((FmsSubsystem.isRedAlliance() && coralPose.getX() > Units.inchesToMeters(630))
        || (!FmsSubsystem.isRedAlliance() && coralPose.getX() < Units.inchesToMeters(55))) {
      return false;
    }

    return true;
  }

  public static boolean isLollipopInSafeSpotForAuto(Translation2d coralPose) {

    if ((FmsSubsystem.isRedAlliance() && coralPose.getX() < Units.inchesToMeters(603))
        || (!FmsSubsystem.isRedAlliance() && coralPose.getX() > Units.inchesToMeters(72))) {
      return false;
    }

    return true;
  }

  private List<Translation2d> getFilteredCoralPoses() {
    if (!safeToTrack()) {
      return List.of();
    }

    var rawCoralPoses = getRawCoralPoses();
    if (DriverStation.isTeleop()) {
      return rawCoralPoses;
    }
    List<Translation2d> safeCoralPoses = new ArrayList<>();
    rawCoralPoses.stream()
        .forEach(
            element -> {
              if (isCoralInSafeSpotForAuto(element)) {
                safeCoralPoses.add(element);
              }
            });

    return safeCoralPoses;
  }

  private boolean safeToTrack() {
    return swerveSpeeds.vxMetersPerSecond < SWERVE_MAX_LINEAR_SPEED_TRACKING
        && swerveSpeeds.vyMetersPerSecond < SWERVE_MAX_LINEAR_SPEED_TRACKING
        && swerveSpeeds.omegaRadiansPerSecond
            < Units.degreesToRadians(SWERVE_MAX_ANGULAR_SPEED_TRACKING);
  }

  private void updateMap() {
    List<Translation2d> filteredCoralPoses = getFilteredCoralPoses();

    coralMap.removeIf(
        element -> {
          return (element.expiresAt() < Timer.getFPGATimestamp());
        });

    if (staleCoralCorners) {
      return;
    }

    double newCoralExpiry = Timer.getFPGATimestamp() + CORAL_LIFETIME_SECONDS;

    for (var visionCoral : filteredCoralPoses) {
      Optional<CoralMapElement> match =
          coralMap.stream()
              .filter(
                  rememberedCoral -> {
                    return rememberedCoral.expiresAt() != newCoralExpiry
                        && (rememberedCoral.coralTranslation().getDistance(visionCoral) < 0.8);
                  })
              .min(
                  (a, b) ->
                      Double.compare(
                          a.coralTranslation().getDistance(visionCoral),
                          b.coralTranslation().getDistance(visionCoral)));

      if (match.isPresent()) {
        coralMap.remove(match.orElseThrow());
      }

      coralMap.add(new CoralMapElement(newCoralExpiry, visionCoral));
    }
  }

  @Override
  protected void collectInputs() {
    if (!FeatureFlags.CORAL_DETECTION.getAsBoolean()) {
      return;
    }
    swerveSpeeds = swerve.getRobotRelativeSpeeds();
    updateMap();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    if (filteredLollipopPose.isPresent()) {
      DogLog.log("CoralMap/LollipopPose", filteredLollipopPose.get());
    } else {
      DogLog.log("CoralMap/LollipopPose", Pose2d.kZero);
    }
    if (!FeatureFlags.CORAL_DETECTION.getAsBoolean()) {
      return;
    }
    try {
      DogLog.log(
          "CoralMap/Map",
          coralMap.stream()
              .map(element -> new Pose2d(element.coralTranslation(), Rotation2d.kZero))
              .toArray(Pose2d[]::new));
    } catch (RuntimeException error) {
      DogLog.logFault("CoralMapLoggingError");
      System.err.println(error);
    }
  }
}
