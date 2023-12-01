package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareAdapter;

public class NetworkTables extends SubsystemBase implements HardwareAdapter{
    public NetworkTables(){
        NetworkTableInstance.getDefault().getTable("turnPID").getEntry("P").setDouble(0);
        NetworkTableInstance.getDefault().getTable("turnPID").getEntry("I").setDouble(0);
        NetworkTableInstance.getDefault().getTable("turnPID").getEntry("D").setDouble(0);
        NetworkTableInstance.getDefault().getTable("Angle").getEntry("Angle").setDouble(0);
    }

    @Override
    public void periodic() {
        NetworkTableInstance.getDefault().getTable("currentAngle").getEntry(SwerveModule.moduleID + "angle").setDouble(SwerveModule.turnEncoder.getAbsolutePosition());


    }

    public void angle(){
        for (var i = 0; i < 4; i++)
            NetworkTableInstance.getDefault().getTable("currentAngle").getEntry(SwerveDrive.swerveModuleOrder[i] + "angle").setDouble(SwerveModule.turnEncoder.getAbsolutePosition());

    }
    
}