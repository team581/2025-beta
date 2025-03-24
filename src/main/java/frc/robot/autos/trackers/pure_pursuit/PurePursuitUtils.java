package frc.robot.autos.trackers.pure_pursuit;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autos.AutoPoint;
import java.util.List;

public class PurePursuitUtils {
  public static final double DYNAMIC_LOOKAHEAD_MAX = 1.5;
  private static final double DYNAMIC_LOOKAHEAD_SCALE = 0.35;

  public static Pose2d getPerpendicularPoint(Pose2d startPoint, Pose2d endPoint, Pose2d robotPose) {
    var x1 = startPoint.getX();
    var y1 = startPoint.getY();

    var x2 = endPoint.getX();
    var y2 = endPoint.getY();

    var x3 = robotPose.getX();
    var y3 = robotPose.getY();

    if (y1 == y2) {
      return new Pose2d(x3, y1, Rotation2d.kZero);
    }
    if (x1 == x2) {
      return new Pose2d(x1, y3, Rotation2d.kZero);
    }
    // Find the slope of the path and y-int
    double pathSlope = (y2 - y1) / (x2 - x1);
    double yInt = y1 - pathSlope * x1;

    // Find the slope and y-int of the perpendicular line
    double perpSlope = -1 / pathSlope;
    double perpYInt = y3 - perpSlope * x3;

    // Calculate the perpendicular intersection
    double perpX = (perpYInt - yInt) / (pathSlope - perpSlope);
    double perpY = pathSlope * perpX + yInt;

    return new Pose2d(perpX, perpY, Rotation2d.kZero);
  }

  public static double getDynamicLookaheadDistance(
      Pose2d firstPoint, Pose2d secondPoint, Pose2d thirdPoint) {
    var x1 = firstPoint.getX();
    var y1 = firstPoint.getY();
    var x2 = secondPoint.getX();
    var y2 = secondPoint.getY();
    var x3 = thirdPoint.getX();
    var y3 = thirdPoint.getY();

    var AB = Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    var BC = Math.sqrt((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    var AC = Math.sqrt((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    var s = (AB + BC + AC) / 2;
    var area = Math.sqrt(s * (s - AB) * (s - BC) * (s - AC));
    if (area == 0) {
      return DYNAMIC_LOOKAHEAD_MAX;
    }
    var curvature = (AB * BC * AC) / (4 * area);
    double value = curvature * DYNAMIC_LOOKAHEAD_SCALE;
    Pose2d[] curvaturepoints = {firstPoint, secondPoint, thirdPoint};
    DogLog.log("Autos/Trailblazer/PurePursuitPathTracker/CurvaturePoints", curvaturepoints);
    return Math.min(value, DYNAMIC_LOOKAHEAD_MAX);
  }

  public static Pose2d getLookaheadPoint(
      Pose2d startPoint, Pose2d endPoint, Pose2d pointOnPath, double lookaheadDistance) {
    var x1 = startPoint.getX();
    var y1 = startPoint.getY();

    var x2 = endPoint.getX();
    var y2 = endPoint.getY();

    var x = pointOnPath.getX();
    var y = pointOnPath.getY();

    var xLookahead =
        x
            + lookaheadDistance
                * ((x2 - x1) / Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2)));
    var yLookahead =
        y
            + lookaheadDistance
                * ((y2 - y1) / Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2)));
    var lookahead = new Pose2d(xLookahead, yLookahead, Rotation2d.kZero);
    var distanceToStart = lookahead.getTranslation().getDistance(startPoint.getTranslation());
    var distanceToEnd = lookahead.getTranslation().getDistance(endPoint.getTranslation());
    var lookaheadOutside =
        !((lookahead.getX() - startPoint.getX()) * (lookahead.getX() - endPoint.getX()) <= 0
            && (lookahead.getY() - startPoint.getY()) * (lookahead.getY() - endPoint.getY()) <= 0);
    if (lookaheadOutside) {
      if (distanceToEnd > distanceToStart) {
        return startPoint;
      }
    }
    return getPerpendicularPoint(startPoint, endPoint, lookahead);
  }

  public static double randomBetween(double min, double max) {
    return Math.random() * (max - min) + min;
  }

  public static Pose2d generateRandomPose() {
    return new Pose2d(
        randomBetween(0, 15),
        randomBetween(0, 8),
        Rotation2d.fromRadians(randomBetween(0, Math.PI * 2)));
  }

  public static Pose2d getTargetPose(
      Pose2d currentPose,
      List<AutoPoint> points,
      int currentPointIndex,
      double lookaheadDistance,
      Pose2d startingRobotPose) {
    var lastTargetWaypoint = Pose2d.kZero;
    var currentTargetWaypoint = Pose2d.kZero;

    if (points.isEmpty()) {
      return Pose2d.kZero;
    }
    if (currentPointIndex == 0) {
      lastTargetWaypoint = startingRobotPose;
    } else {
      lastTargetWaypoint = points.get(currentPointIndex - 1).poseSupplier.get();
    }
    currentTargetWaypoint = points.get(currentPointIndex).poseSupplier.get();
    var perpendicularPoint =
        getPerpendicularPoint(lastTargetWaypoint, currentTargetWaypoint, currentPose);

    var lookaheadPoint =
        getLookaheadPoint(
            lastTargetWaypoint,
            currentTargetWaypoint,
            perpendicularPoint,
            lookaheadDistance
                - currentPose.getTranslation().getDistance(perpendicularPoint.getTranslation()));
    var lookaheadInside = isBetween(lastTargetWaypoint, currentTargetWaypoint, lookaheadPoint);
    var lookaheadToStartDistance =
        lookaheadPoint.getTranslation().getDistance(lastTargetWaypoint.getTranslation());
    var lookaheadToEndDistance =
        lookaheadPoint.getTranslation().getDistance(currentTargetWaypoint.getTranslation());
    if (!lookaheadInside) {
      if (lookaheadToEndDistance > lookaheadToStartDistance) {
        return new Pose2d(
            lastTargetWaypoint.getTranslation(),
            getPointToPointInterpolatedRotation(
                lastTargetWaypoint,
                currentTargetWaypoint,
                getPerpendicularPoint(lastTargetWaypoint, currentTargetWaypoint, currentPose)));
      }
      if (currentPointIndex < points.size() - 1) {
        var futurePoint = points.get(currentPointIndex + 1).poseSupplier.get();
        var perpendicularToCurrentEndDistance =
            perpendicularPoint.getTranslation().getDistance(currentTargetWaypoint.getTranslation());
        var newLookaheadPoint =
            getLookaheadPoint(
                currentTargetWaypoint,
                futurePoint,
                currentTargetWaypoint,
                lookaheadDistance - perpendicularToCurrentEndDistance);

        currentPointIndex++;
        var newLookaheadInside = isBetween(currentTargetWaypoint, futurePoint, newLookaheadPoint);
        if (!newLookaheadInside) {
          return new Pose2d(
              currentTargetWaypoint.getTranslation(),
              getPointToPointInterpolatedRotation(
                  currentPose,
                  futurePoint,
                  getPerpendicularPoint(currentTargetWaypoint, futurePoint, currentPose)));
        }
        return newLookaheadPoint;
      } else {
        return new Pose2d(
            currentTargetWaypoint.getTranslation(),
            getPointToPointInterpolatedRotation(
                lastTargetWaypoint,
                currentTargetWaypoint,
                getPerpendicularPoint(lastTargetWaypoint, currentTargetWaypoint, currentPose)));
      }
    }
    return lookaheadPoint;
  }

  public static Rotation2d getPointToPointInterpolatedRotation(
      Pose2d startPoint, Pose2d endPoint, Pose2d pointOnPath) {
    var totalDistance = startPoint.getTranslation().getDistance(endPoint.getTranslation());
    if (totalDistance < 0.0001) {
      return endPoint.getRotation();
    }
    var pointToStart = pointOnPath.getTranslation().getDistance(startPoint.getTranslation());
    var pointToEnd = pointOnPath.getTranslation().getDistance(endPoint.getTranslation());

    if (!((pointOnPath.getX() - startPoint.getX()) * (pointOnPath.getX() - endPoint.getX()) <= 0
        && (pointOnPath.getY() - startPoint.getY()) * (pointOnPath.getY() - endPoint.getY())
            <= 0)) {
      if (pointToEnd > pointToStart) {
        return startPoint.getRotation();
      } else {
        return endPoint.getRotation();
      }
    }
    var progressPercent = Math.abs((pointToStart / totalDistance) * 1.2);

    var interpolatedRotation =
        startPoint.getRotation().interpolate(endPoint.getRotation(), progressPercent);

    return interpolatedRotation;
  }

  /**
   * Checks if a Pose2d lies between two other Pose2ds.
   *
   * @param startPose The starting Pose2d.
   * @param endPose The ending Pose2d.
   * @param poseOnPath The Pose2d to check if it lies between startPose and endPose.
   * @return True if poseOnPath lies between startPose and endPose
   */
  public static boolean isBetween(Pose2d startPose, Pose2d endPose, Pose2d poseOnPath) {
    // Extract x and y coordinates
    double startX = startPose.getX();
    double startY = startPose.getY();
    double endX = endPose.getX();
    double endY = endPose.getY();
    double poseX = poseOnPath.getX();
    double poseY = poseOnPath.getY();
    double tolerance = 0.001;

    // Check if poseOnPath is within the bounds of startPose and endPose along both axes
    // with tolerance
    return ((startX - tolerance <= poseX && poseX <= endX + tolerance)
            || (endX - tolerance <= poseX && poseX <= startX + tolerance))
        && ((startY - tolerance <= poseY && poseY <= endY + tolerance)
            || (endY - tolerance <= poseY && poseY <= startY + tolerance));
  }

  /**
   * Checks if a Pose2d is collinear with two other Pose2ds.
   *
   * @param startPose The starting Pose2d.
   * @param endPose The ending Pose2d.
   * @param poseOnPath The Pose2d to check if it lies collinear with startPose and endPose.
   * @return True if poseOnPath lies collinear with startPose and endPose
   */
  public static boolean isCollinear(Pose2d startPose, Pose2d endPose, Pose2d poseOnPath) {
    // Extract x and y coordinates
    double startX = startPose.getX();
    double startY = startPose.getY();
    double endX = endPose.getX();
    double endY = endPose.getY();
    double poseX = poseOnPath.getX();
    double poseY = poseOnPath.getY();

    double area = startX * (endY - poseY) + endX * (poseY - startY) + poseX * (startY - endY);
    return Math.abs(area) < 0.001;
  }

  public static boolean isBetweenAndCollinearWithAnyPoints(
      Pose2d startingRobotPose, List<AutoPoint> points, Pose2d poseOnPath) {
    if (isBetween(startingRobotPose, points.get(0).poseSupplier.get(), poseOnPath)
        && isCollinear(startingRobotPose, points.get(0).poseSupplier.get(), poseOnPath)) {
      return true;
    }
    for (int i = 0; i < points.size() - 1; i++) {
      var startPose = points.get(i).poseSupplier.get();
      var endPose = points.get(i + 1).poseSupplier.get();
      if (isBetween(startPose, endPose, poseOnPath)
          && isCollinear(startPose, endPose, poseOnPath)) {
        return true;
      }
    }

    System.out.println(
        "DogLog.log(\"PurePursuitUtils/CollinearAndBetweenTest/Points\", new Pose2d[] {");

    for (AutoPoint point : points) {
      System.out.println(
          "  new Pose2d("
              + point.poseSupplier.get().getX()
              + ", "
              + point.poseSupplier.get().getY()
              + ", "
              + "Rotation2d.kZero),");
    }

    System.out.println("});");

    System.out.println(
        "DogLog.log(\"PurePursuitUtils/CollinearAndBetweenTest/MockRobotPoint\", new Pose2d("
            + poseOnPath.getX()
            + ", "
            + poseOnPath.getY()
            + ", Rotation2d.kZero));");
    System.out.println(
        "DogLog.log(\"PurePursuitUtils/CollinearAndBetweenTest/StartingRobotPose\", new Pose2d("
            + startingRobotPose.getX()
            + ", "
            + startingRobotPose.getY()
            + ", Rotation2d.kZero));");

    return false;
  }

  private PurePursuitUtils() {}
}
