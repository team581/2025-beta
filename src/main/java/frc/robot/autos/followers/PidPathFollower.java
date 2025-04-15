package frc.robot.autos.followers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PidPathFollower implements PathFollower {
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;

  public PidPathFollower(
      PIDController xController, PIDController yController, PIDController thetaController) {
    this.xController = xController;
    this.yController = yController;
    this.thetaController = thetaController;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public ChassisSpeeds calculateSpeeds(
      Pose2d currentPose, ChassisSpeeds currentSpeeds, Pose2d targetPose) {
    return new ChassisSpeeds(
        xController.calculate(currentPose.getX(), targetPose.getX()),
        yController.calculate(currentPose.getY(), targetPose.getY()),
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()));
  }
}
