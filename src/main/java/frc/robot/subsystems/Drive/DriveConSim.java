package frc.robot.subsystems.Drive;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveConSim implements DriveConMoudle{
    DCMotorSim leftDriveMotor = new DCMotorSim(DCMotor.getCIM(2), 7.86, 10); 
    DCMotorSim rightDriveMotor = new DCMotorSim(DCMotor.getCIM(2), 7.86, 10); 

    EncoderSim leftEncoderSim = new EncoderSim(new Encoder(0, 1));
    EncoderSim rightEncoderSim = new EncoderSim( new Encoder(2, 3));

    Rotation2d angle = new Rotation2d(0);

    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0);
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
        leftDriveMotor.update(0.02);

        leftEncoderSim.setDistance(leftDriveMotor.getAngularPositionRotations());
    }

    @Override
    public void setRightPower(double speed){

        rightDriveMotor.setInputVoltage(speed * RobotController.getInputVoltage());
        rightDriveMotor.update(0.02);

        rightEncoderSim.setDistance(rightDriveMotor.getAngularPositionRotations());
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

        public Pose3d getPose(){

            SmartDashboard.putNumber("Left encoder", leftEncoderSim.getDistance());
            SmartDashboard.putNumber("Right Encoder", rightEncoderSim.getDistance());


            odometry.update(new Rotation2d(0), leftEncoderSim.getDistance(), rightEncoderSim.getDistance());

            return new Pose3d(odometry.getPoseMeters());
        }
}
