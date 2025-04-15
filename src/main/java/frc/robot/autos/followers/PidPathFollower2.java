package frc.robot.autos.followers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.intake_assist.IntakeAssistUtil;

public class PidPathFollower2 implements PathFollower {
  private final PIDController distanceController;
  private final PIDController angleController;

  public PidPathFollower2(PIDController distanceController, PIDController angleController) {
    this.distanceController = distanceController;
    this.angleController = angleController;

    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public ChassisSpeeds calculateSpeeds(
      Pose2d currentPose, ChassisSpeeds currentSpeeds, Pose2d targetPose) {
    var distanceToGoal = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    var wantedLinearVelocity = Math.abs(distanceController.calculate(distanceToGoal, 0));
    var wantedAngularVelocity =
        angleController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    var robotDirection =
        Math.toRadians(
            IntakeAssistUtil.getIntakeAssistAngle(targetPose.getTranslation(), currentPose));
    return new ChassisSpeeds(
        wantedLinearVelocity * Math.cos(robotDirection),
        wantedLinearVelocity * Math.sin(robotDirection),
        wantedAngularVelocity);
  }
}
