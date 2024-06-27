// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("Pose", Pose2d.struct).publish();

  DriveConMoudle driveCon = new DriveConSim();

  /** Creates a new Drive. */
  public Drive() {}

  public void DiffDrive(double y, double omega){
    driveCon.DiffDrive(y, omega);
  }

  public Rotation2d getRotation(){
    return driveCon.getRotation();
  }

  public Pose2d getPose(){
    return driveCon.getPose();
  }

  @Override
  public void periodic() {


    publisher.set(getPose());
    // This method will be called once per scheduler run
  }
}
