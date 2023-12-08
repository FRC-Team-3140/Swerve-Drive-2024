package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareAdapter;

//Everything in this class is  measured in meters
public class SwerveDrive extends SubsystemBase implements HardwareAdapter {
    public static String[] swerveModuleOrder = { "frontLeft",
            "frontRight", "backLeft", "backRight" };
    double maxDriveSpeed;
    double maxTurnSpeed;
    boolean locked = false;
    private final Translation2d[] locations = { new Translation2d(0.254, 0.254), new Translation2d(0.254, -0.254),
            new Translation2d(-0.254, 0.254), new Translation2d(-0.254, -0.254) };

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(locations);

    SwerveModule[] modules = {
            new SwerveModule("frontLeft", 0, 2, 1, 0.969279),
            new SwerveModule("frontRight", 1, 4, 3, 0.207867),
            new SwerveModule("backLeft", 2, 6, 5, 0.697409),
            new SwerveModule("backRight", 3, 8, 7, 0.701239),
    };

    public SwerveDrive() {
        // Configure the AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                this // Reference to this subsystem to set requirements
        );
    }

    public void setChassisSpeeds(double xSpeed, double ySpeed, double rads_per_sec, boolean headless) {
        SwerveModuleState[] states = new SwerveModuleState[4];
        if (headless) {
            ChassisSpeeds botSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rads_per_sec,
                    gyro.getRotation2d());
            states = kinematics.toSwerveModuleStates(botSpeeds);
        } else {
            ChassisSpeeds fieldSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rads_per_sec);
            states = kinematics.toSwerveModuleStates(fieldSpeeds);
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxDriveSpeed);
        for (int i = 0; i < 4; i++) {
            modules[i].setStates(states[i], locked);
        }
    }

    public void setLockState(boolean locked) {
        this.locked = locked;
    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        return null;
    }

    private void driveRobotRelative(ChassisSpeeds chassisspeeds1) {
    }

    private void resetPose(Pose2d pose2d1) {
    }

    private Pose2d getPose() {
        // This code was just finished to remove errors, but it isn't correct.
        Pose2d temp = new Pose2d();
        return temp;
    }
}