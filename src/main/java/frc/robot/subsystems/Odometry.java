// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public final class Odometry {
    private final static AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    public Odometry(SwerveDrive drivetrain) {
        // initialize the singleton
        m_drive_train = drivetrain;
        odometer = new SwerveDriveOdometry(m_drive_train.getKinematics(), m_drive_train.getYaw(),
                m_drive_train.getModulePositions(), new Pose2d());
    }

    public static Pose2d getEstimatedGlobalPose(Pose2d estimatedRobotPose) {
        var rand = StateSpaceUtil.makeWhiteNoiseVector(VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
        return new Pose2d(
                estimatedRobotPose.getX() + rand.get(0, 0),
                estimatedRobotPose.getY() + rand.get(1, 0),
                estimatedRobotPose.getRotation().plus(new Rotation2d(rand.get(2, 0))));
    }

    public static AHRS getGyro() {
        return m_gyro;
    }
}