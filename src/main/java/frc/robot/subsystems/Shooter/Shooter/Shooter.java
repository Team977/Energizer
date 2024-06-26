// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MotorConst;
import static edu.wpi.first.units.Units.*;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

   ShooterConModlue shooterCon = new ShooterConSim();


  public Shooter() {}


  public void setSpeed(double speed){
    SmartDashboard.putNumber("Target Speed", speed);
    shooterCon.setSpeed(speed);
  }

  public Measure<Velocity<Angle>> getVelocity(){
    return shooterCon.getVelcocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current RPS", getVelocity().in(RotationsPerSecond));
    // This method will be called once per scheduler run
  }
}
