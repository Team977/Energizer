// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Aim.Aim;

public class setAimPosition extends Command {

  DoubleSupplier aimPos;
  Aim aim;

  ProfiledPIDController aimController
    = new ProfiledPIDController(0.025, 0, 0,
        new TrapezoidProfile.Constraints(1, 0.2));


  //PIDController aimController = new PIDController(0.00004, 0, 0);
  /** Creates a new setAimPosition. */
  public setAimPosition(DoubleSupplier aimPos, Aim aim) {

    this.aimPos = aimPos;
    this.aim = aim;

    addRequirements(aim);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    aim.setSpeed(aimController.calculate(aim.getRotation().getDegrees(), aimPos.getAsDouble()));

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
