package frc.robot.subsystems.Drive;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface DriveConMoudle {


    public void DiffDrive(double y, double omega);

    public Rotation2d getRotation();

    public Pose2d getPose();



    
}
