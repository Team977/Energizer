// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakePnewmatics;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePnewmatics extends SubsystemBase {
  static final IntakePnewmaticsConMoudlue INTAKE_PNEWMATICS_CON = new IntakePnewmaticsConSIm();

  boolean Forward = false;
  /** Creates a new IntakePnewmatics. */
  public IntakePnewmatics() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Phewmatics Position(1 fully out, 0 in)", forward() ? 1 : 0);
    // This method will be called once per scheduler run
  }

  public void Switch(){
    Forward = INTAKE_PNEWMATICS_CON.Switch();
  }

  public boolean forward(){
    return Forward;
  }
}
