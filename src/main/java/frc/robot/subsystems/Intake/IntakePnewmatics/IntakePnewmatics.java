// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakePnewmatics;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeConst;
import frc.robot.subsystems.Intake.IntakeWheal.IntakeWhealsConSim;

public class IntakePnewmatics extends SubsystemBase {
  static final IntakePnewmaticsConMoudlue INTAKE_PNEWMATICS_CON = new IntakePnewmaticsConSIm();

  boolean Forward = false;
  /** Creates a new IntakePnewmatics. */
  public IntakePnewmatics() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Switch(){
    Forward = INTAKE_PNEWMATICS_CON.Switch();
  }

  public boolean forward(){
    return Forward;
  }
}
