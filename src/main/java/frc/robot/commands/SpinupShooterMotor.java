// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter.Shooter;

public class SpinupShooterMotor extends Command {

  DoubleSupplier rotationsPerSec;
  Shooter shooter;

  PIDController pid = new PIDController(40, 0, 0.1);

  /** Creates a new SpinupShooterMotor. */
  public SpinupShooterMotor(DoubleSupplier rotationsPerSecound, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.

    rotationsPerSec = rotationsPerSecound;
    this.shooter = shooter;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.setSpeed(pid.calculate(shooter.getVelocity().in(RotationsPerSecond)
      , rotationsPerSec.getAsDouble()));

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
