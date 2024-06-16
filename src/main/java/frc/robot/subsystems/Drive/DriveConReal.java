package frc.robot.subsystems.Drive;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import frc.robot.subsystems.MotorConst;
import frc.robot.subsystems.Drive.DriveConMoudle;

public class DriveConReal implements DriveConMoudle{
    PWMVictorSPX leftDriveMotor = new PWMVictorSPX(MotorConst.driveLeftChannle);
    PWMVictorSPX RightDriveMotor = new PWMVictorSPX(MotorConst.driveRightChannle);

    AHRS gyro = new AHRS(SPI.Port.kMXP);

    @Override
    public void setPower(double LeftSpeed, double RightSpeed){
        setLeftPower(LeftSpeed);
        setRightPower(RightSpeed);
    }

    @Override
    public void setLeftPower(double speed){
        leftDriveMotor.set(speed);
    }

    @Override
    public void setRightPower(double speed){
        RightDriveMotor.set(speed);
    }

    @Override
    public Rotation2d getRotation(){
        return new Rotation2d(Units.degreesToRadians(gyro.getAngle()));
    }

    @Override
    public GyroData getGyroData(){
        GyroData gyroData = new GyroData();
        gyroData.AccleX = gyro.getRawAccelX();
        gyroData.AccleY = gyro.getRawAccelY();
        gyroData.AccleZ = gyro.getRawAccelZ();
        gyroData.yaw = getRotation();
        return gyroData;
    }
}
