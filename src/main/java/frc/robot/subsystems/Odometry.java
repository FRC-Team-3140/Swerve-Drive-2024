package frc.robot.subsystems;

// import com.pathplanner.lib.path.PathPlannerPath;

public class Odometry {
    public double currentPositionX = 0;
    public double currentPositionY = 0;
    // private PathPlannerPath path = null;

    public void resetPos() {
        currentPositionX = 0;
        currentPositionY = 0;
    }

    public void getPath(String pathToOdometryPath) {
        // path = PathPlannerPath.fromPathFile(pathToOdometryPath);
    }

    public void followSelectedPath() {
        /*if (path != null) {
            PathPlannerPath.FollowPathWithEvents()
        }*/
    }

}