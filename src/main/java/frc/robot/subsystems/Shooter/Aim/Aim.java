// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Aim;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Aim extends SubsystemBase {

  AimMotorConMoudle aimMotorConMoudle;

  Rotation2d aimOffset = new Rotation2d(0);


  /** Creates a new Aim. */
  public Aim() {
    aimMotorConMoudle = new AimMotorConSim();
  }

  public void setSpeed(double speed){
    //SmartDashboard.putNumber("Aim Motor Desired Speed", speed);
    aimMotorConMoudle.setSpeed(speed);
  }

  public Measure<Velocity<Angle>> getVelocity(){
    return aimMotorConMoudle.getVelcocity();
    //return RotationsPerSecond.of(ENCODER.getVelocity()/60);
  }

  public Rotation2d getRotation(){
    return new Rotation2d(
      Units.rotationsToRadians(
          getRawRotation().getRotations() + aimOffset.getRotations())
      
    );
  }

  public Rotation2d getRawRotation(){
    return aimMotorConMoudle.getRawRotation();
  }

  public void setRotationOffset(Rotation2d rotation){
    this.aimOffset = rotation;
  }

  public double getVolts(){
    return aimMotorConMoudle.getVolts();
  }

  public double getCurrent(){
    return aimMotorConMoudle.getCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Aimer Rotation", getRotation().getDegrees());
    //SmartDashboard.putNumber("Aim Motor Velocity", getVelocity().in(DegreesPerSecond));
    // This method will be called once per scheduler run
  }
}
