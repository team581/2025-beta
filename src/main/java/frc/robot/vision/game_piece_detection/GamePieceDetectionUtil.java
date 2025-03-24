package frc.robot.vision.game_piece_detection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig;
import frc.robot.intake_assist.IntakeAssistUtil;
import frc.robot.vision.results.GamePieceResult;

public final class GamePieceDetectionUtil {

  private static final double CORAL_LENGTH = 11.875;
  private static final double CORAL_RADIUS = 2.25;
  private static final double ALGAE_DIAMETER = 16.25;
  private static final double algaeToGroundOffset =
      CORAL_LENGTH + (ALGAE_DIAMETER / 2.0); // length of coral + half of diameter of algae

  private static final Transform3d LOLLIPOP_OFFSET =
      new Transform3d(0, 0, Units.inchesToMeters(-algaeToGroundOffset), Rotation3d.kZero);

  private static final Transform3d HORIZONTAL_CORAL_OFFSET =
      new Transform3d(0, 0, Units.inchesToMeters(-CORAL_RADIUS), Rotation3d.kZero);

  private static final Pose3d LIMELIGHT_POSE_TO_ROBOT_WITH_ALGAE_OFFSET =
      RobotConfig.get().vision().rightLimelightPosition().transformBy(LOLLIPOP_OFFSET);

  private static final Pose3d LIMELIGHT_POSE_TO_ROBOT_WITH_CORAL_OFFSET =
      RobotConfig.get().vision().rightLimelightPosition().transformBy(HORIZONTAL_CORAL_OFFSET);

  public static Translation2d calculateFieldRelativeCoralTranslationFromCamera(
      Pose2d robotPoseAtCapture, GamePieceResult visionResult) {
    var robotRelative =
        calculateRobotRelativeTranslationFromCamera(
            visionResult, LIMELIGHT_POSE_TO_ROBOT_WITH_CORAL_OFFSET);
    return robotRelativeToFieldRelativeGamePiecePose(robotPoseAtCapture, robotRelative);
  }

  public static Translation2d calculateFieldRelativeLollipopTranslationFromCamera(
      Pose2d robotPoseAtCapture, GamePieceResult visionResult) {
    var robotRelative =
        calculateRobotRelativeTranslationFromCamera(
            visionResult, LIMELIGHT_POSE_TO_ROBOT_WITH_ALGAE_OFFSET);
    return robotRelativeToFieldRelativeGamePiecePose(robotPoseAtCapture, robotRelative);
  }

  public static double getFieldRelativeAngleToCoral(
      Pose2d robotPoseAtCapture, GamePieceResult visionResult) {
    var gamePiecePose =
        calculateFieldRelativeCoralTranslationFromCamera(robotPoseAtCapture, visionResult);
    return IntakeAssistUtil.getIntakeAssistAngle(gamePiecePose, robotPoseAtCapture);
  }

  public static double getFieldRelativeAngleToLollipop(
      Pose2d robotPoseAtCapture, GamePieceResult visionResult) {
    var gamePiecePose =
        calculateFieldRelativeLollipopTranslationFromCamera(robotPoseAtCapture, visionResult);
    return IntakeAssistUtil.getIntakeAssistAngle(gamePiecePose, robotPoseAtCapture);
  }

  public static Translation2d calculateRobotRelativeLollipopTranslationFromCamera(
      Pose2d robotPoseAtCapture, GamePieceResult visionResult) {
    return calculateRobotRelativeTranslationFromCamera(
        visionResult, LIMELIGHT_POSE_TO_ROBOT_WITH_ALGAE_OFFSET);
  }

  public static Translation2d calculateRobotRelativeTranslationFromCamera(
      GamePieceResult visionResult, Pose3d limelightToRobotOffset) {

    double thetaX = Units.degreesToRadians(visionResult.tx());
    double thetaY = Units.degreesToRadians(visionResult.ty());
    double hypot = Math.copySign(Math.hypot(thetaX, thetaY), thetaX);
    double thetaRelativeToCenter = Math.atan(thetaY / thetaX);
    double adjustedRelativeToCenter =
        thetaRelativeToCenter + limelightToRobotOffset.getRotation().getX();
    double newThetaX = -1 * (hypot * Math.cos(adjustedRelativeToCenter));
    double newThetaY = hypot * Math.sin(adjustedRelativeToCenter);

    double adjustedThetaY = limelightToRobotOffset.getRotation().getY() - newThetaY;

    double forwardOffset = 0;
    if (adjustedThetaY == 0) {
      forwardOffset = Math.abs(limelightToRobotOffset.getY());
    } else {
      forwardOffset =
          // .getZ() represents height from floor
          (limelightToRobotOffset.getZ() / Math.tan(adjustedThetaY));
    }

    double sidewaysOffset = forwardOffset * Math.tan(newThetaX);

    var cameraRelativeTranslation = new Translation2d(forwardOffset, sidewaysOffset);
    var robotRelativeTranslation =
        cameraRelativeTranslation
            .rotateBy(new Rotation2d(limelightToRobotOffset.getRotation().getZ()))
            .plus(limelightToRobotOffset.getTranslation().toTranslation2d());
    return robotRelativeTranslation;
  }

  public static Translation2d robotRelativeToFieldRelativeGamePiecePose(
      Pose2d robotPose, Translation2d robotRelativeGamePiecePose) {
    return robotRelativeGamePiecePose
        .rotateBy(robotPose.getRotation())
        .plus(robotPose.getTranslation());
  }

  private GamePieceDetectionUtil() {}
}
