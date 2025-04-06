package frc.robot.robot_manager.collision_avoidance;

import com.google.common.collect.ImmutableList;
import com.google.common.graph.ElementOrder;
import com.google.common.graph.ImmutableValueGraph;
import com.google.common.graph.MutableValueGraph;
import com.google.common.graph.ValueGraphBuilder;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.arm.ArmState;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.ArrayDeque;
import java.util.Comparator;
import java.util.Deque;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Stream;

public class CollisionAvoidance {
  private static final double ELEVATOR_TOLERANCE = 25.0;
  private static final double ARM_TOLERANCE = 20.0;
  private static final double CLIMBER_UNSAFE_ANGLE = 225.0;

  private static final ImmutableValueGraph<Waypoint, WaypointEdge> graph = createGraph();

  private static final Map<CollisionAvoidanceQuery, Optional<ImmutableList<Waypoint>>> aStarCache =
      new HashMap<>();

  private static CollisionAvoidanceQuery lastQuery =
      new CollisionAvoidanceQuery(
          Waypoint.ELEVATOR_0_ARM_UP, Waypoint.ELEVATOR_0_ARM_UP, ObstructionKind.NONE);
  private static double lastSolution = 90.0;
  private static double lastgoalAngle = 90;
  private static boolean lastClimberRisky = true;
  private static ObstructionKind lastObstruction = ObstructionKind.NONE;
  private static ObstructionStrategy lastLeftStrategy = ObstructionStrategy.IGNORE_BLOCKED;
  private static ObstructionStrategy lastRightStrategy = ObstructionStrategy.IGNORE_BLOCKED;

  private static Deque<Waypoint> lastPath = new ArrayDeque<>();

  private static boolean hasGeneratedPath = false;
  private static Waypoint previousWaypoint;

  /**
   * Returns an {@link Optional} containing the next {@link Waypoint} in the graph to go to. Returns
   * an empty Optional if there is no possible routing (impossible to avoid a collision or you are
   * at final waypoint).
   *
   * @param currentPosition The current position of the superstructure.
   * @param desiredPosition The desired position of the superstructure.
   * @param obstructionKind Additional constraints based on robot position.
   */
  public static Optional<SuperstructurePosition> routePosition(
      SuperstructurePosition currentPosition,
      SuperstructurePosition desiredPosition,
      ObstructionKind obstructionKind,
      double rawArmAngle) {
    double armGoal;
    var maybeWaypoint = route(currentPosition, desiredPosition, obstructionKind);
    if (maybeWaypoint.isEmpty()) {
      return Optional.empty();
    }
    Waypoint waypoint = maybeWaypoint.get();
    var maybeEdge = graph.edgeValue(previousWaypoint, waypoint);
    if (maybeEdge.isEmpty()) {
      return Optional.empty();
    }
    var edge = maybeEdge;

    if (waypoint.position.armAngle() != lastgoalAngle
        || edge.get().hitsClimber() != lastClimberRisky
        || obstructionKind != lastObstruction
        || edge.get().leftSideStrategy() != lastLeftStrategy
        || edge.get().rightSideStrategy() != lastRightStrategy) {
      armGoal =
          getCollisionAvoidanceAngleGoal(
              waypoint.position.armAngle(),
              edge.get().hitsClimber(),
              obstructionKind,
              edge.get().leftSideStrategy(),
              edge.get().rightSideStrategy(),
              rawArmAngle);
      lastgoalAngle = waypoint.position.armAngle();
      lastClimberRisky = edge.get().hitsClimber();
      lastObstruction = obstructionKind;
      lastLeftStrategy = edge.get().leftSideStrategy();
      lastRightStrategy = edge.get().rightSideStrategy();
      lastSolution = armGoal;
    }
    armGoal = lastSolution;

    DogLog.log(
        "CollisionAvoidance/CollisionAvoidanceAngleVariables/goalAngle",
        waypoint.position.armAngle());
    DogLog.log(
        "CollisionAvoidance/CollisionAvoidanceAngleVariables/hitsClimber",
        edge.get().hitsClimber());

    DogLog.log(
        "CollisionAvoidance/CollisionAvoidanceAngleVariables/obstructionKind", obstructionKind);

    DogLog.log(
        "CollisionAvoidance/CollisionAvoidanceAngleVariables/leftStrat",
        edge.get().leftSideStrategy());
    DogLog.log(
        "CollisionAvoidance/CollisionAvoidanceAngleVariables/rightStrat",
        edge.get().rightSideStrategy());
    DogLog.log("CollisionAvoidance/CollisionAvoidanceAngleVariables/RawArmAngle", rawArmAngle);
    DogLog.log("CollisionAvoidance/CollisionAvoidanceAngleVariables/solution", armGoal);

    return Optional.of(new SuperstructurePosition(waypoint.position.elevatorHeight(), armGoal));
  }

  public static Optional<Waypoint> route(
      SuperstructurePosition currentPosition,
      SuperstructurePosition desiredPosition,
      ObstructionKind obstructionKind) {
    DogLog.log("CollisionAvoidance/ClawPos", currentPosition.getTranslation());

    if (DriverStation.isDisabled()) {
      return Optional.empty();
    }
    if (Waypoint.getClosest(currentPosition) == Waypoint.getClosest(desiredPosition)) {
      return Optional.empty();
    }
    DogLog.log("CollisionAvoidance/DesiredWaypoint", Waypoint.getClosest(desiredPosition));
    // Check if the desired position and obstruction is the same, then use the same path
    if (!lastQuery.goalWaypoint().equals(Waypoint.getClosest(desiredPosition))
        || !lastQuery.obstructionKind().equals(obstructionKind)) {
      var currentWaypoint = Waypoint.getClosest(currentPosition);
      lastQuery =
          new CollisionAvoidanceQuery(
              currentWaypoint, Waypoint.getClosest(desiredPosition), obstructionKind);

      var maybePath = cachedAStar(lastQuery).map(ArrayDeque::new);
      if (maybePath.isPresent()) {
        hasGeneratedPath = true;
        lastPath = maybePath.orElseThrow();
      } else {
        return Optional.empty();
      }
    }

    if (lastPath.isEmpty()) {
      return Optional.empty();
    }

    var currentWaypoint = lastPath.getFirst();
    DogLog.log(
        "CollisionAvoidance/CurrentWaypoint/ElevatorHeight",
        currentWaypoint.position.elevatorHeight());
    DogLog.log("CollisionAvoidance/CurrentWaypoint/ArmAngle", currentWaypoint.position.armAngle());
    DogLog.log("CollisionAvoidance/CurrentWaypoint", currentWaypoint);
    DogLog.log("CollisionAvoidance/ClosestWaypoint", Waypoint.getClosest(currentPosition));
    DogLog.log("CollisionAvoidance/AstarPath", lastPath.toArray(Waypoint[]::new));

    DogLog.log(
        "CollisionAvoidance/CurrentPosition/ElevatorHeight", currentPosition.elevatorHeight());
    DogLog.log("CollisionAvoidance/CurrentPosition/ArmAngle", currentPosition.armAngle());
    DogLog.log("CollisionAvoidance/Obstruction", obstructionKind);

    boolean near =
        currentPosition.isNear(currentWaypoint.position, ELEVATOR_TOLERANCE, ARM_TOLERANCE);
    DogLog.log("CollisionAvoidance/Near", near);

    // Check if our current position is close to the current waypoint in path
    if (near) {
      // If it's close, return the next waypoint
      if (lastPath.isEmpty()) {
        return Optional.empty();
      }
      previousWaypoint = currentWaypoint;

      return Optional.of(lastPath.pop());
    }
    // If it's not close, return the same waypoint
    return Optional.of(currentWaypoint);
  }

  public static double getCollisionAvoidanceAngleGoal(
      double angle,
      boolean climberRisky,
      ObstructionKind currentObstructionKind,
      ObstructionStrategy leftObstructionStrategy,
      ObstructionStrategy rightObstructionStrategy,
      double currentRawMotorAngle) {

    double solution1;
    double solution2;
    double shortSolution;
    double longSolution;
    double collisionAvoidanceGoal;

    var wrap = (int) Math.floor(currentRawMotorAngle / 360.0);

    //   double angleWrapped = MathUtil.inputModulus(angle,0,360);
    //   double difference = Math.abs(Math.max(angleWrapped,
    // MathUtil.inputModulus(currentRawMotorAngle, 0, 360))-Math.min(angleWrapped,
    // MathUtil.inputModulus(currentRawMotorAngle, 0, 360)));

    //   System.out.println("solution1 diff = "+difference);

    //   solution1 = currentRawMotorAngle+difference;
    //  solution2 =currentRawMotorAngle-(360-difference);

    //   System.out.println("solution1  = "+solution1);
    //   System.out.println("solution2 = "+solution2);
    if (angle < 0) {
      solution1 = (wrap * 360) - Math.abs(angle);
      solution2 = (wrap * 360) + (360 - Math.abs(angle));
    } else {
      solution1 = (wrap * 360) + angle;
      solution2 = (wrap * 360) - (360 - angle);
    }

    double climberUnsafeAngle1 = (wrap * 360) - (360 - CLIMBER_UNSAFE_ANGLE);
    double climberUnsafeAngle2 = (wrap * 360) + CLIMBER_UNSAFE_ANGLE;

    if (climberRisky) {
      if ((Math.max(currentRawMotorAngle, solution1) >= climberUnsafeAngle1
              && Math.min(currentRawMotorAngle, solution1) <= climberUnsafeAngle1)
          || (Math.max(currentRawMotorAngle, solution1) >= climberUnsafeAngle2
              && Math.min(currentRawMotorAngle, solution1)
                  <= climberUnsafeAngle2)) { // bad spot is in between the solution 1 path
        collisionAvoidanceGoal = solution2;
      } else if ((Math.max(currentRawMotorAngle, solution2) > climberUnsafeAngle1
              && Math.min(currentRawMotorAngle, solution2) < climberUnsafeAngle1)
          || (Math.max(currentRawMotorAngle, solution2) > climberUnsafeAngle2
              && Math.min(currentRawMotorAngle, solution2)
                  < climberUnsafeAngle2)) { // bad spot is in between the solution 2 path
        collisionAvoidanceGoal = solution1;
      } else {
        collisionAvoidanceGoal = currentRawMotorAngle; // Something very bad has happened
      }

    } else {
      // System.out.println("1 = "+solution1);
      // System.out.println("2 = "+solution2);
      // double solution1Difference = Math.abs(Math.max(solution1,
      // currentRawMotorAngle)-Math.min(solution1, currentRawMotorAngle));
      // double solution2Difference = Math.abs(Math.max(solution2,
      // currentRawMotorAngle)-Math.min(solution2, currentRawMotorAngle));
      // System.out.println("sol 1 diff"+solution1Difference);
      // System.out.println("sol 2 diff"+solution2Difference);

      //       if (solution2Difference > solution1Difference) {
      if (Math.abs(solution2 - currentRawMotorAngle) > Math.abs(solution1 - currentRawMotorAngle)) {
        shortSolution = solution1;
        longSolution = solution2;
      } else {
        shortSolution = solution2;
        longSolution = solution1;
      }
      System.out.println("shortSolution = " + shortSolution);
      System.out.println("longsolution = " + longSolution);

      collisionAvoidanceGoal =
          switch (currentObstructionKind) {
            case LEFT_OBSTRUCTED ->
                switch (leftObstructionStrategy) {
                  case IGNORE_BLOCKED -> shortSolution;
                  case IMPOSSIBLE_IF_BLOCKED -> currentRawMotorAngle;
                  case LONG_WAY_IF_BLOCKED -> longSolution;
                };
            case RIGHT_OBSTRUCTED ->
                switch (rightObstructionStrategy) {
                  case IGNORE_BLOCKED -> shortSolution;
                  case IMPOSSIBLE_IF_BLOCKED -> currentRawMotorAngle;
                  case LONG_WAY_IF_BLOCKED -> longSolution;
                };
            default -> shortSolution;
          };
    }
    return collisionAvoidanceGoal;
  }

  private static Optional<ImmutableList<Waypoint>> cachedAStar(CollisionAvoidanceQuery query) {
    DogLog.log("CollisionAvoidance/AStarCacheSize", aStarCache.size());

    return aStarCache.computeIfAbsent(
        query,
        (k) ->
            aStar(
                query.currentWaypoint().position,
                query.goalWaypoint().position,
                query.obstructionKind()));
  }

  private static ImmutableValueGraph<Waypoint, WaypointEdge> createGraph() {
    // Create an undirected value graph to represent safe motion between waypoints. Undirected
    // because if you can go from A to B, you can also go from B to A. Value graph because we want
    // to associate a cost with motion between different waypoints.
    MutableValueGraph<Waypoint, WaypointEdge> graph =
        ValueGraphBuilder.undirected().incidentEdgeOrder(ElementOrder.stable()).build();

    // Try to categorize blocks based on the kinds of collision that may happen
    // Sort L2 before L3, L3 before L4
    // Always have left on the left, and right on the right

    /* If your arm angle doesn't change, you can do whatever with elevator */
    var armStraightUpWaypoints =
        Stream.of(Waypoint.values())
            .filter(waypoint -> waypoint.position.armAngle() == ArmState.HOLDING_UPRIGHT.getAngle())
            .toList();
    var armStraightDownWaypoints =
        Stream.of(Waypoint.values())
            .filter(waypoint -> waypoint.position.armAngle() == ArmState.CORAL_HANDOFF.getAngle())
            .toList();

    for (var a : armStraightUpWaypoints) {
      for (var b : armStraightUpWaypoints) {
        if (a == b) {
          // Skip because it's the same waypoint
          continue;
        }

        a.alwaysSafe(graph, b);
      }
    }

    for (var a : armStraightDownWaypoints) {
      for (var b : armStraightDownWaypoints) {
        if (a == b) {
          // Skip because it's the same waypoint
          continue;
        }

        a.alwaysSafe(graph, b);
      }
    }

    // TODO: Need to see if it's actually safe to do this all in one move
    Waypoint.HANDOFF.avoidClimberAlwaysSafe(graph, Waypoint.ELEVATOR_0_ARM_UP);
    Waypoint.HANDOFF_CLEARS_CLIMBER.avoidClimberAlwaysSafe(graph, Waypoint.ELEVATOR_0_ARM_UP);
    // TODO: Previously this had ALGAE_NET_UP.avoidClimberAlwaysSafe(HANDOFF_CLEARS_CLIMBER), seems
    // like that bonks the net?

    /* Arm up to left/right is always safe */
    Waypoint.L2_UPRIGHT.avoidClimberAlwaysSafe(
        graph,
        Waypoint.L2_LEFT_LINEUP,
        Waypoint.L2_RIGHT_LINEUP,
        Waypoint.GROUND_ALGAE_INTAKE,
        Waypoint.LOLLIPOP_INTAKE_RIGHT,
        Waypoint.REEF_ALGAE_L2_LEFT,
        Waypoint.REEF_ALGAE_L3_LEFT,
        Waypoint.REEF_ALGAE_L2_RIGHT,
        Waypoint.REEF_ALGAE_L3_RIGHT);
    Waypoint.L3_UPRIGHT.avoidClimberAlwaysSafe(
        graph,
        Waypoint.L3_LEFT_LINEUP,
        Waypoint.L3_RIGHT_LINEUP,
        Waypoint.GROUND_ALGAE_INTAKE,
        Waypoint.LOLLIPOP_INTAKE_RIGHT,
        Waypoint.REEF_ALGAE_L2_LEFT,
        Waypoint.REEF_ALGAE_L3_LEFT,
        Waypoint.REEF_ALGAE_L2_RIGHT,
        Waypoint.REEF_ALGAE_L3_RIGHT);
    Waypoint.L4_UPRIGHT.avoidClimberAlwaysSafe(
        graph,
        Waypoint.L4_LEFT_LINEUP,
        Waypoint.L4_RIGHT_LINEUP,
        Waypoint.GROUND_ALGAE_INTAKE,
        Waypoint.LOLLIPOP_INTAKE_RIGHT,
        Waypoint.REEF_ALGAE_L2_LEFT,
        Waypoint.REEF_ALGAE_L3_LEFT,
        Waypoint.REEF_ALGAE_L2_RIGHT,
        Waypoint.REEF_ALGAE_L3_RIGHT);
    Waypoint.ALGAE_NET_UP.alwaysSafe(
        graph, Waypoint.ALGAE_NET_OUT_LEFT, Waypoint.ALGAE_NET_OUT_RIGHT);

    // If you aren't going to hit reef poles, you can skip the in between upright waypoints
    Waypoint.L2_UPRIGHT.leftSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L3_LEFT_LINEUP,
        Waypoint.L4_LEFT_LINEUP);
    Waypoint.L3_UPRIGHT.leftSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L2_LEFT_LINEUP,
        Waypoint.L4_LEFT_LINEUP);
    Waypoint.L4_UPRIGHT.leftSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L2_LEFT_LINEUP,
        Waypoint.L3_LEFT_LINEUP);

    Waypoint.L2_UPRIGHT.rightSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L1_RIGHT_LINEUP,
        Waypoint.L3_RIGHT_LINEUP,
        Waypoint.L4_RIGHT_LINEUP);
    Waypoint.L3_UPRIGHT.rightSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L1_RIGHT_LINEUP,
        Waypoint.L2_RIGHT_LINEUP,
        Waypoint.L4_RIGHT_LINEUP);
    Waypoint.L4_UPRIGHT.rightSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L1_RIGHT_LINEUP,
        Waypoint.L2_RIGHT_LINEUP,
        Waypoint.L3_RIGHT_LINEUP);

    /* Scoring coral (arm left to arm right) */
    Waypoint.L2_LEFT_LINEUP.alwaysSafe(graph, Waypoint.L2_RIGHT_LINEUP);

    Waypoint.L3_LEFT_LINEUP.alwaysSafe(graph, Waypoint.L3_RIGHT_LINEUP);

    Waypoint.L4_LEFT_LINEUP.alwaysSafe(graph, Waypoint.L4_RIGHT_LINEUP);

    /* Lineup to place states can always happen since the arm is already out */
    // Waypoint.L2_LEFT_LINEUP.alwaysSafe(graph, Waypoint.L2_LEFT_PLACE);
    // Waypoint.L3_LEFT_LINEUP.alwaysSafe(graph, Waypoint.L3_LEFT_PLACE);
    // Waypoint.L4_LEFT_LINEUP.alwaysSafe(graph, Waypoint.L4_LEFT_PLACE);

    // Waypoint.L2_RIGHT_LINEUP.alwaysSafe(graph, Waypoint.L2_RIGHT_PLACE);
    // Waypoint.L3_RIGHT_LINEUP.alwaysSafe(graph, Waypoint.L3_RIGHT_PLACE);
    // Waypoint.L4_RIGHT_LINEUP.alwaysSafe(graph, Waypoint.L4_RIGHT_PLACE);

    /* Scoring coral directly from handoff, depends a lot on obstructions */
    // TODO: Make sure the elevator doesn't go down before the arm can get safe
    // TODO: Make sure the HANDOFF_BUT_HIGHER is safe to go to left side scoring states and avoid
    // climber
    Waypoint.HANDOFF_CLEARS_CLIMBER.leftSideSpecial(
        graph,
        ObstructionStrategy.LONG_WAY_IF_BLOCKED,
        Waypoint.L2_LEFT_LINEUP,
        Waypoint.L3_LEFT_LINEUP,
        Waypoint.L4_LEFT_LINEUP);
    Waypoint.HANDOFF_CLEARS_CLIMBER.rightSideSpecial(
        graph,
        ObstructionStrategy.LONG_WAY_IF_BLOCKED,
        Waypoint.L2_RIGHT_LINEUP,
        Waypoint.L3_RIGHT_LINEUP,
        Waypoint.L4_RIGHT_LINEUP);

    // /* Finish score, go back to handoff, depends a lot on obstructions */
    // // TODO: Make sure we can stow to HANDOFF_CLEARS_CLIMBER from each place position
    // Waypoint.HANDOFF_CLEARS_CLIMBER.avoidClimberAlwaysSafe(
    //     graph,Waypoint.L2_LEFT_PLACE, Waypoint.L3_LEFT_PLACE, Waypoint.L4_LEFT_PLACE);
    // Waypoint.HANDOFF_CLEARS_CLIMBER.avoidClimberAlwaysSafe(
    //     graph,Waypoint.L2_RIGHT_PLACE, Waypoint.L3_RIGHT_PLACE, Waypoint.L4_RIGHT_PLACE);

    // /* Place to reef algae intake */
    // Waypoint.REEF_ALGAE_L2_LEFT.alwaysSafe(
    //     graph, Waypoint.L2_LEFT_PLACE, Waypoint.L3_LEFT_PLACE, Waypoint.L4_LEFT_PLACE);
    // Waypoint.REEF_ALGAE_L3_LEFT.alwaysSafe(
    //     graph, Waypoint.L2_LEFT_PLACE, Waypoint.L3_LEFT_PLACE, Waypoint.L4_LEFT_PLACE);

    // Waypoint.REEF_ALGAE_L2_RIGHT.alwaysSafe(
    //     graph, Waypoint.L2_RIGHT_PLACE, Waypoint.L3_RIGHT_PLACE, Waypoint.L4_LEFT_PLACE);
    // Waypoint.REEF_ALGAE_L3_RIGHT.alwaysSafe(
    //     graph, Waypoint.L2_RIGHT_PLACE, Waypoint.L3_RIGHT_PLACE, Waypoint.L4_LEFT_PLACE);

    // L1 movements
    Waypoint.L1_RIGHT_LINEUP.alwaysSafe(graph, Waypoint.ELEVATOR_0_ARM_UP);
    // Ground algae movement
    Waypoint.GROUND_ALGAE_INTAKE.avoidClimberAlwaysSafe(graph, Waypoint.HANDOFF_CLEARS_CLIMBER);

    // Create an immutable copy of the graph now that we've added all the nodes
    var immutableGraph = ImmutableValueGraph.copyOf(graph);

    // Visualize the generated graph
    Waypoint.log();

    return immutableGraph;
  }

  /**
   * Returns an {@link Optional} containing the next {@link Waypoint} in the graph to go to. Returns
   * an empty Optional if there is no possible routing (impossible to avoid a collision or you are
   * at final waypoint).
   */
  private static ImmutableList<Waypoint> reconstructPath(
      Map<Waypoint, Waypoint> cameFrom, Waypoint endWaypoint) {
    Deque<Waypoint> totalPath = new ArrayDeque<Waypoint>();
    totalPath.add(endWaypoint);
    Waypoint current = endWaypoint;
    while (cameFrom.containsKey(current)) {
      current = cameFrom.get(current);
      totalPath.addFirst(current);
    }

    return ImmutableList.copyOf(totalPath);
  }

  static Optional<ImmutableList<Waypoint>> aStar(
      SuperstructurePosition currentPosition,
      SuperstructurePosition desiredPosition,
      ObstructionKind obstructionKind) {
    var openSet = EnumSet.of(Waypoint.getClosest(currentPosition));

    Map<Waypoint, Waypoint> cameFrom = new EnumMap<Waypoint, Waypoint>(Waypoint.class);

    Map<Waypoint, Double> gscore = new EnumMap<Waypoint, Double>(Waypoint.class);

    Waypoint startWaypoint = Waypoint.getClosest(currentPosition);
    Waypoint goalWaypoint = Waypoint.getClosest(desiredPosition);
    if (startWaypoint.equals(goalWaypoint)) {
      DogLog.timestamp("CollisionAvoidance/StartAndEndSame");
      return Optional.empty();
    }

    gscore.put(startWaypoint, 0.0);
    Waypoint current = Waypoint.ELEVATOR_0_ARM_UP;
    while (!openSet.isEmpty()) {
      // current is equal to the waypoint in openset that has the smallest gscore
      var maybeCurrent =
          openSet.stream()
              .min(
                  Comparator.comparingDouble(
                      waypoint -> gscore.getOrDefault(waypoint, Double.MAX_VALUE)));
      if (maybeCurrent.isPresent()) {
        current = maybeCurrent.orElseThrow();
      }

      if (current == goalWaypoint) {
        var totalPath = reconstructPath(cameFrom, current);
        DogLog.clearFault("Collision avoidance path not possible");
        return Optional.of(totalPath);
      }
      openSet.remove(current);
      Set<Waypoint> options = graph.adjacentNodes(current);
      // Set<Waypoint> placeException =
      // Set.of(Waypoint.L2_LEFT_PLACE,Waypoint.L3_LEFT_PLACE,Waypoint.L4_LEFT_PLACE,Waypoint.L2_RIGHT_PLACE,Waypoint.L3_RIGHT_PLACE,Waypoint.L4_RIGHT_PLACE);
      // Set<Waypoint> lineUpException =
      // Set.of(Waypoint.L2_LEFT_LINEUP,Waypoint.L3_LEFT_LINEUP,Waypoint.L4_LEFT_LINEUP,Waypoint.L2_RIGHT_LINEUP,Waypoint.L3_RIGHT_LINEUP,Waypoint.L4_RIGHT_LINEUP);

      // if(placeException.contains(current)&&options.contains(lineUpException))

      for (Waypoint neighbor : options) {
        var edge = graph.edgeValue(current, neighbor);
        double tentativeGScore =
            gscore.getOrDefault(current, Double.MAX_VALUE)
                + edge.orElseThrow().getCost(obstructionKind);
        if (tentativeGScore < gscore.getOrDefault(neighbor, Double.MAX_VALUE)) {
          cameFrom.put(neighbor, current);
          gscore.put(neighbor, tentativeGScore);
          openSet.add(neighbor);
        }
      }
    }
    DogLog.logFault("Collision avoidance path not possible", AlertType.kWarning);
    return Optional.of(ImmutableList.of(Waypoint.getClosest(currentPosition)));
  }

  /** Don't use this. */
  static ImmutableValueGraph<Waypoint, WaypointEdge> getRawGraph() {
    return graph;
  }

  public CollisionAvoidance() {}
}
