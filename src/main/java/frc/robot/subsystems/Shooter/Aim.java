// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;
import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MotorConst;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;

public class Aim extends SubsystemBase {

  Rotation2d aimOffset = new Rotation2d(0);

  static final CANSparkMax AimMotor = new CANSparkMax(MotorConst.AimMotorCANId, MotorType.kBrushless);
  static final RelativeEncoder ENCODER = AimMotor.getEncoder();
  /** Creates a new Aim. */
  public Aim() {}

  public void setSpeed(double speed){
    AimMotor.set(speed);
  }

  public Measure<Velocity<Angle>> getVelocity(){
    return RotationsPerSecond.of(ENCODER.getVelocity()/60);
  }

  public Rotation2d getRotation(){
    return new Rotation2d(
      Units.rotationsToRadians(
          ENCODER.getPosition() + aimOffset.getRotations())
      
    );
  }

  public Rotation2d getRawRotation(){
    return new Rotation2d(
      Units.rotationsToRadians(
        ENCODER.getPosition()
      )
    );
  }

  public void setRotationOffset(Rotation2d rotation){
    this.aimOffset = rotation;
  }

  public double getVolts(){
    return AimMotor.getAppliedOutput() * AimMotor.getBusVoltage();
  }

  public double getCurrent(){
    return AimMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
