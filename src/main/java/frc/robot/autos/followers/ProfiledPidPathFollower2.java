package frc.robot.autos.followers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autos.constraints.AutoConstraintOptions;

public class ProfiledPidPathFollower2 implements PathFollower {
  private final PidPathFollower2 follower;
  private final ProfiledPIDController distanceController;
  private final ProfiledPIDController angleController;

  public ProfiledPidPathFollower2(PIDController distanceController, PIDController angleController) {
    var defaultConstraints = new AutoConstraintOptions();
    // TODO: ProfiledPIDController uses theoretical speeds (last output of controller) instead of
    // actual measured speeds, might want to DIY our own that doesn't have that limitation
    this.distanceController =
        new ProfiledPIDController(
            distanceController.getP(),
            distanceController.getI(),
            distanceController.getD(),
            defaultConstraints.getLinearConstraints());
    this.angleController =
        new ProfiledPIDController(
            angleController.getP(),
            angleController.getI(),
            angleController.getD(),
            defaultConstraints.getAngularConstraints());

    angleController.enableContinuousInput(-Math.PI, Math.PI);

    follower = new PidPathFollower2(distanceController, angleController);
  }

  @Override
  public ChassisSpeeds calculateSpeeds(
      Pose2d currentPose, ChassisSpeeds currentSpeeds, Pose2d targetPose) {
    return follower.calculateSpeeds(currentPose, currentSpeeds, targetPose);
  }

  @Override
  public void setConstraints(AutoConstraintOptions constraints) {
    distanceController.setConstraints(constraints.getLinearConstraints());
    angleController.setConstraints(constraints.getAngularConstraints());
  }

  @Override
  public boolean handlesConstraints() {
    return true;
  }
}
