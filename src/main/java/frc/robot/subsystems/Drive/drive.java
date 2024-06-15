// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MotorConst;
import edu.wpi.first.units.*;

public class drive extends SubsystemBase {

  PWMVictorSPX leftDriveMotor = new PWMVictorSPX(MotorConst.driveLeftChannle);
  PWMVictorSPX RightDriveMotor = new PWMVictorSPX(MotorConst.driveRightChannle);

  DifferentialDrive differentialDrive;

  AHRS gyro = new AHRS(SPI.Port.kMXP);

  /** Creates a new drive. */
  public drive() {
    differentialDrive = new DifferentialDrive(
      (LeftPower) -> {
        leftDriveMotor.set(LeftPower);
      }, 
      (RightPower) -> {
        leftDriveMotor.set(RightPower);
      });

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void DiffDrive(Double y, Double Omega){
    differentialDrive.arcadeDrive(y, Omega);
  }

  public Rotation2d getYaw(){
    return new Rotation2d(Units.Degrees.of(gyro.getAngle()));
  }

  public double getYawDeg(){
    return gyro.getAngle();
  }
}
