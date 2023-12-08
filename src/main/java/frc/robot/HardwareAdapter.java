package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SwerveDrive;

public interface HardwareAdapter {
    public AHRS gyro = new AHRS(SPI.Port.kMXP);
    // public SwerveDrive swerveDrive = new SwerveDrive();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    public final XboxController xbox = new XboxController(0);
}
