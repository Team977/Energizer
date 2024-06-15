// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Aim;

public class getAimOffset extends Command {

  Aim aim;

  double peekCurrent;
  double speed;
  double peekCurrentAimOffset;

  /** Creates a new getAimOffset. */
  public getAimOffset() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(aim.getCurrent() > peekCurrent){
      aim.setSpeed(0);
      aim.setRotationOffset(
        new Rotation2d(Degrees.of(peekCurrentAimOffset - aim.getRawRotation().getDegrees()))
      );

      return true;
    }

    return false;
  }
}
