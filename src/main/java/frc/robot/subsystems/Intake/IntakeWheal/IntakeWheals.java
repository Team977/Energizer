// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakeWheal;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWheals extends SubsystemBase {

  IntakeWhealsConModlue whealsCon = new IntakeWhealsConSim();

  /** Creates a new IntakeWheals. */
  public IntakeWheals() {}

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Intake Wheale Vel SIM", whealsCon.getVelcocity());
    // This method will be called once per scheduler run
  }

  public void runMotor(double speed){
    whealsCon.setSpeed(speed);
  }
}
