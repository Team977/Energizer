// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpinDexer.Spindexer;

public class SpinUpSoinDexer extends Command {

  DoubleSupplier speed;
  Spindexer spindexer;
  PIDController pid = new PIDController(0.01, 0, 0);

  /** Creates a new SpinUpSoinDexer. */
  public SpinUpSoinDexer(DoubleSupplier speed,Spindexer spindexer) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.speed = speed;
    this.spindexer = spindexer;
    addRequirements(spindexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spindexer.setSpeed(pid.calculate(spindexer.getSpeed(), speed.getAsDouble()));
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
