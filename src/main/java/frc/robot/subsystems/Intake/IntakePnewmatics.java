// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePnewmatics extends SubsystemBase {
  static final DoubleSolenoid leftDoubleSolenoid 
    = new DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      IntakeConst.LeftIntakeSOlChannle1, 
      IntakeConst.LeftIntakeSOlChannle2);

  static final DoubleSolenoid rightDoubleSolenoid 
    = new DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      IntakeConst.RightIntakeSOlChannle1, 
      IntakeConst.RightIntakeSOlChannle2);

  boolean Forward;

  /** Creates a new IntakePnewmatics. */
  public IntakePnewmatics() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Switch(){
    if (Forward) {

      leftDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      rightDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);

      Forward = false;
    }else{

      leftDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
      rightDoubleSolenoid.set(DoubleSolenoid.Value.kForward);

      Forward = true;
    }
  }

  public boolean forward(){
    return Forward;
  }
}
