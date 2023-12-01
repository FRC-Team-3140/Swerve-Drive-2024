package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.HardwareAdapter;

public class Odometry {
    public double currentPositionX = 0;
    public double currentPositionY = 0;
    private PathPlannerPath path = null;

    public void resetPos() {
        currentPositionX = 0;
        currentPositionY = 0;
    }

    public void getPath(String pathToOdometryPath) {
        path = PathPlannerPath.fromPathFile(pathToOdometryPath);
    }

    public void followSelectedPath() {
        if (path != null) {
            PathPlannerPath.FollowPathWithEvents()
        }
    }

}