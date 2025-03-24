package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.util.PoseErrorTolerance;
import java.util.List;

/**
 * A segment is a path (a continuous set of {@link AutoPoint points}) that the roobt will follow.
 */
public class AutoSegment {
  private static final PoseErrorTolerance DEFAULT_POSITION_TOLERANCE =
      new PoseErrorTolerance(0.1, 5);

  public final List<AutoPoint> points;

  /**
   * Constraints to apply to any points that don't have their own constraints specified. If a point
   * specifies its own constraints, this field will be ignored.
   */
  public final AutoConstraintOptions defaultConstraints;

  public final PoseErrorTolerance positionTolerance;

  public AutoSegment(
      AutoConstraintOptions defaultConstraints,
      PoseErrorTolerance positionTolerance,
      List<AutoPoint> points) {
    this.defaultConstraints = defaultConstraints;
    this.points = points;
    this.positionTolerance = positionTolerance;
  }

  public AutoSegment(
      AutoConstraintOptions defaultConstraints,
      PoseErrorTolerance positionTolerance,
      AutoPoint... points) {
    this(defaultConstraints, positionTolerance, List.of(points));
  }

  public AutoSegment(AutoConstraintOptions defaultConstraints, AutoPoint... points) {
    this(defaultConstraints, DEFAULT_POSITION_TOLERANCE, List.of(points));
  }

  public AutoSegment(AutoPoint... points) {
    this(new AutoConstraintOptions(), DEFAULT_POSITION_TOLERANCE, points);
  }

  public AutoSegment(PoseErrorTolerance positionTolerance, AutoPoint... points) {
    this(new AutoConstraintOptions(), positionTolerance, points);
  }

  public AutoSegment pathflipped() {
    return new AutoSegment(
        defaultConstraints,
        positionTolerance,
        points.stream().map(AutoPoint::pathflipped).toList());
  }

  /**
   * Get the remaining distance from the robot's current pose to the end of the segment.
   *
   * @param robotPose The current pose of the robot.
   * @param currentIndex The current index of the segment.
   */
  public double getRemainingDistance(Pose2d robotPose, int currentIndex) {
    var remainingPoints = points.subList(currentIndex, points.size());

    var distance = 0.0;

    for (var i = 0; i < remainingPoints.size(); i++) {
      var previous = i == 0 ? robotPose : remainingPoints.get(i - 1).poseSupplier.get();
      var current = remainingPoints.get(i).poseSupplier.get();

      distance += previous.getTranslation().getDistance(current.getTranslation());
    }

    return distance;
  }

  public boolean isFinished(Pose2d robotPose, int currentIndex) {
    if (points.isEmpty()) {
      return true;
    }

    if (currentIndex != points.size() - 1) {
      // We aren't at the last point in the list, so we definitely aren't finished
      return false;
    }

    return positionTolerance.atPose(points.get(points.size() - 1).poseSupplier.get(), robotPose);
  }
}
