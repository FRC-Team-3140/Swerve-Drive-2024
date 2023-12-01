package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareAdapter;

//Everything in this class is  measured in meters
public class SwerveDrive extends SubsystemBase implements HardwareAdapter {
    private final String[] swerveModuleOrder = { "frontLeft",
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

}
