package frc.robot.util;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


public class LocalADStarAK implements Pathfinder {
  private final ADStarIO io = new ADStarIO();

  /**
   * Get if a new path has been calculated since the last time a path was retrieved.
   *
   * @return True if a new path is available.
   */
  @Override
  public boolean isNewPathAvailable() {
    io.updateIsNewPathAvailable();
    return io.isNewPathAvailable;
  }

  /**
   * Get the most recently calculated path.
   *
   * @param constraints The path constraints to use when creating the path.
   * @param goalEndState The goal end state to use when creating the path.
   * @return The PathPlannerPath created from the points calculated by the pathfinder.
   */
  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
    io.updateCurrentPathPoints(constraints, goalEndState);
    if (io.currentPathPoints.isEmpty()) {
      return null;
    }
    return PathPlannerPath.fromPathPoints(io.currentPathPoints, constraints, goalEndState);
  }

  /**
   * Set the start position to pathfind from.
   *
   * @param startPosition Start position on the field. If this is within an obstacle it will be
   *                      moved to the nearest non-obstacle node.
   */
  @Override
  public void setStartPosition(Translation2d startPosition) {
    io.adStar.setStartPosition(startPosition);
  }

  /**
   * Set the goal position to pathfind to.
   *
   * @param goalPosition Goal position on the field. If this is within an obstacle it will be moved
   *                     to the nearest non-obstacle node.
   */
  @Override
  public void setGoalPosition(Translation2d goalPosition) {
    io.adStar.setGoalPosition(goalPosition);
  }

  /**
   * Set the dynamic obstacles that should be avoided while pathfinding.
   *
   * @param obs              A List of Translation2d pairs representing obstacles. Each Translation2d
   *                         represents opposite corners of a bounding box.
   * @param currentRobotPos  The current position of the robot. This is needed to change the start
   *                         position of the path to properly avoid obstacles.
   */
  @Override
  public void setDynamicObstacles(
      List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
    io.adStar.setDynamicObstacles(obs, currentRobotPos);
  }

  private static class ADStarIO {
    public LocalADStar adStar = new LocalADStar();
    public boolean isNewPathAvailable = false;
    public List<PathPoint> currentPathPoints = Collections.emptyList();



    public void updateIsNewPathAvailable() {
      isNewPathAvailable = adStar.isNewPathAvailable();
    }

    public void updateCurrentPathPoints(PathConstraints constraints, GoalEndState goalEndState) {
      PathPlannerPath currentPath = adStar.getCurrentPath(constraints, goalEndState);
      if (currentPath != null) {
        currentPathPoints = currentPath.getAllPathPoints();
      } else {
        currentPathPoints = Collections.emptyList();
      }
    }
  }
}
