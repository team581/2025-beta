package frc.robot.intake_assist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig;
import frc.robot.vision.game_piece_detection.GamePieceDetectionUtil;
import frc.robot.vision.results.GamePieceResult;
import java.util.Optional;

public final class IntakeAssistUtil {
  private static final double CORAL_ASSIST_KP = 3.0;
  private static final double INTAKE_OFFSET = Units.inchesToMeters(18);

  public static ChassisSpeeds getAssistSpeedsFromPose(
      Pose2d target, Pose2d robotPose, ChassisSpeeds teleopSpeeds) {
    var robotRelativePose =
        target
            .getTranslation()
            .minus(robotPose.getTranslation())
            .rotateBy(Rotation2d.fromDegrees(360 - robotPose.getRotation().getDegrees()));
    var sidewaysSpeed = robotRelativePose.getY();
    var forwardError = Math.hypot(teleopSpeeds.vxMetersPerSecond, teleopSpeeds.vyMetersPerSecond);
    var robotRelativeError = new Translation2d(forwardError, sidewaysSpeed * CORAL_ASSIST_KP);
    var fieldRelativeError = robotRelativeError.rotateBy(robotPose.getRotation());
    var assistSpeeds =
        new ChassisSpeeds(fieldRelativeError.getX(), fieldRelativeError.getY(), 0.0).times(0.8);
    var scaledTeleopSpeeds = teleopSpeeds.times(0.2);
    return assistSpeeds.plus(scaledTeleopSpeeds);
  }

  public static double getIntakeAssistAngle(Translation2d target, Pose2d robotPose) {
    return Units.radiansToDegrees(
        Math.atan2(target.getY() - robotPose.getY(), target.getX() - robotPose.getX()));
  }

  public static Optional<Pose2d> getLollipopIntakePoseFromVisionResult(
      Optional<GamePieceResult> result, Pose2d robotPose) {
    if (result.isEmpty()) {
      return Optional.empty();
    }
    var translation =
        GamePieceDetectionUtil.calculateRobotRelativeLollipopTranslationFromCamera(
            robotPose, result.get());
    var robotRelativeRotation =
        Rotation2d.fromDegrees(getIntakeAssistAngle(translation, Pose2d.kZero) + 90.0);
    var withRotation = new Pose2d(translation.getX(), translation.getY(), robotRelativeRotation);
    var offset =
        withRotation.transformBy(
            new Transform2d(
                Units.inchesToMeters(-RobotConfig.get().arm().inchesFromCenter()),
                INTAKE_OFFSET,
                Rotation2d.kZero));
    var fieldRelativeIntakePose =
        new Pose2d(
            GamePieceDetectionUtil.robotRelativeToFieldRelativeGamePiecePose(
                robotPose, offset.getTranslation()),
            robotRelativeRotation.plus(robotPose.getRotation()));
    return Optional.of(fieldRelativeIntakePose);
  }

  private IntakeAssistUtil() {}
}
