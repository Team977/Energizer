package frc.robot.subsystems.Drive;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;

public interface DriveConMoudle {

    public class GyroData {
    
        Rotation2d yaw;

        double AccleZ, AccleY, AccleX;
        
    }

    public void setPower(double LeftSpeed, double RightSpeed);
    public void setLeftPower(double speed);
    public void setRightPower(double speed);

    public Rotation2d getRotation();
    public GyroData getGyroData();



    
}
