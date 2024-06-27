// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakePnewmatics.IntakePnewmatics;
import frc.robot.subsystems.Intake.IntakeWheal.IntakeWheals;

public class Intake extends Command {


  private final double IntakeSpeed;
  private final double inSec, OutSec;

  private final IntakePnewmatics PNEWMATICS;
  private final IntakeWheals INTAKE_WHEALS;

  private Timer timer = new Timer();

  /** Creates a new Intake. */
  public Intake(double InSec, double OutSec, double IntakeSpeed, IntakePnewmatics pnewmatics, IntakeWheals intakeWheals) {
  
    this.INTAKE_WHEALS = intakeWheals;
    PNEWMATICS = pnewmatics;
    this.IntakeSpeed = IntakeSpeed;
    this.inSec = InSec;
    this.OutSec = OutSec;

    addRequirements(PNEWMATICS, INTAKE_WHEALS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    INTAKE_WHEALS.runMotor(IntakeSpeed);

    if(PNEWMATICS.forward() && timer.advanceIfElapsed(OutSec)){
      timer.reset();
      PNEWMATICS.Switch();
    }else if(!PNEWMATICS.forward() && timer.advanceIfElapsed(inSec)){
      timer.reset();
      PNEWMATICS.Switch();
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
