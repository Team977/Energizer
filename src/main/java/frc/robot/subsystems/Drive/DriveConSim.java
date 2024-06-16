package frc.robot.subsystems.Drive;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.subsystems.MotorConst;
import frc.robot.subsystems.Drive.DriveConMoudle;

public class DriveConSim implements DriveConMoudle{
    DCMotorSim leftDriveMotor = new DCMotorSim(DCMotor.getCIM(2), 1, 0); 
    DCMotorSim rightDriveMotor = new DCMotorSim(DCMotor.getCIM(2), 1, 0); 

    EncoderSim leftEncoderSim;
    EncoderSim rightEncoderSim;

    Rotation2d angle = new Rotation2d(0);

    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(angle, 0, 0);
  
  DriveConSim(){
        leftEncoderSim.setDistancePerPulse(Math.PI * driveCons.kWhealDiameter / 1000);
        rightEncoderSim.setDistancePerPulse(Math.PI * driveCons.kWhealDiameter / 1000);

    }

    @Override
    public void setPower(double LeftSpeed, double RightSpeed){
    }

    @Override
    public void setLeftPower(double speed){
        leftDriveMotor.setInputVoltage(speed * RobotController.getInputVoltage());
        leftEncoderSim.setDistance(leftDriveMotor.getAngularPositionRotations());
        //leftDriveMotor.set(speed);
    }

    @Override
    public void setRightPower(double speed){
        rightDriveMotor.setInputVoltage(speed * RobotController.getInputVoltage());
        rightEncoderSim.setDistance(rightDriveMotor.getAngularPositionRotations());
        //RightDriveMotor.set(speed);
    }

    @Override
    public Rotation2d getRotation(){
        return angle;
    }

    @Override
    public GyroData getGyroData(){
        GyroData gyroData = new GyroData();
        gyroData.yaw = getRotation();
        return gyroData;
    }
}
