// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.MotorConst;
import edu.wpi.first.units.*;
import edu.wpi.first.util.sendable.Sendable;

public class drive extends SubsystemBase {
  DriveConMoudle driveCon;
  
  DifferentialDrive differentialDrive;

  /** Creates a new drive. */
  public drive() {

    driveCon = new DriveConSim();

    differentialDrive = new DifferentialDrive(
      (LeftPower) -> {
        driveCon.setLeftPower(LeftPower);
      }, 
      (RightPower) -> {
        driveCon.setRightPower(RightPower);
      });

  }

  @Override
  public void periodic() {


    SmartDashboard.putNumberArray("pos", new Double[]{driveCon.getPose().toPose2d().getX(), driveCon.getPose().toPose2d().getY(), driveCon.getPose().toPose2d().getRotation().getDegrees()});


  }

  public void DiffDrive(Double y, Double Omega){
    differentialDrive.arcadeDrive(y, Omega);
  }

  public Rotation2d getYaw(){
    return new Rotation2d(Units.Degrees.of(driveCon.getRotation().getDegrees()));
  }

  public double getYawDeg(){
    return driveCon.getRotation().getDegrees();
  }
}
