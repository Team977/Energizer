package frc.robot.subsystems.Drive;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveConSim implements DriveConMoudle{

    double LeftInput; double RightInput;

    // Create the simulation model of our drivetrain.
    DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
        DCMotor.getCIM(2),       // 2 NEO motors on each side of the drivetrain.
        7.29,                    // 7.29:1 gearing reduction.
        7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
        60.0,                    // The mass of the robot is 60 kg.
        Units.inchesToMeters(3), // The robot uses 3" radius wheels.
        0.7112,                  // The track width is 0.7112 meters.

        // The standard deviations for measurement noise:
        // x and y:          0.001 m
        // heading:          0.001 rad
        // l and r velocity: 0.1   m/s
         // l and r position: 0.005 m
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

    DifferentialDrive driveCon = new DifferentialDrive(
        LeftMotorPower -> {
            LeftInput = LeftMotorPower;
        }, RightMotorPower -> {
            RightInput = RightMotorPower;
        });

    public void DiffDrive(double y, double omega){
        driveCon.arcadeDrive(y, omega);

        m_driveSim.setInputs(LeftInput * RobotController.getInputVoltage(), RightInput * RobotController.getInputVoltage());
        m_driveSim.update(0.02);
    }

    public Rotation2d getRotation(){
        return m_driveSim.getHeading();
    }

    public Pose2d getPose(){
        return m_driveSim.getPose();
    }


}
