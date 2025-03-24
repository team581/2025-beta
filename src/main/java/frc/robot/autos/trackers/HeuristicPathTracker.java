package frc.robot.autos.trackers;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autos.AutoPoint;
import java.util.List;

public class HeuristicPathTracker implements PathTracker {
  private List<AutoPoint> points = List.of();
  private Pose2d currentPose = Pose2d.kZero;
  private static final double PROXIMITY_RADIUS = 0.5;
  private int currentPointIndex = 0;

  @Override
  public void resetAndSetPoints(List<AutoPoint> points) {
    this.points = points;
  }

  @Override
  public void updateRobotState(Pose2d currentPose, ChassisSpeeds currentFieldRelativeRobotSpeeds) {
    this.currentPose = currentPose;
  }

  @Override
  public Pose2d getTargetPose() {
    AutoPoint currentPoint = points.get(getCurrentPointIndex());
    Pose2d currentTargetPose = currentPoint.poseSupplier.get();

    /// DogLog.log("Autos/Trailblazer/CurrentWaypoint", currentPoint);
    DogLog.log("Autos/Trailblazer/NextPointIndex", currentPointIndex);
    DogLog.log("Autos/Trailblazer/CurrentTargetPose", currentTargetPose);

    double distanceToTarget =
        currentPose.getTranslation().getDistance(currentTargetPose.getTranslation());

    if (distanceToTarget < PROXIMITY_RADIUS && currentPointIndex < points.size() - 1) {
      currentPointIndex++;
    }

    var targetPose = points.get(currentPointIndex).poseSupplier.get();

    DogLog.log("Autos/Trailblazer/TargetPose", targetPose);
    DogLog.log("Autos/Trailblazer/DistanceToTarget", distanceToTarget);

    return targetPose;
  }

  @Override
  public int getCurrentPointIndex() {
    return currentPointIndex;
  }
}
