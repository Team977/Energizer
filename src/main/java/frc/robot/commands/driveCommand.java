// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive.Drive;

public class driveCommand{

  private static final double DEADBAND = 0.1;

  public static Command driveCommands(DoubleSupplier y, DoubleSupplier omega, Drive Drive){
    return Commands.run(
      () -> {
        double ProsY = Math.pow( MathUtil.applyDeadband(y.getAsDouble(), DEADBAND), 2);
        double ProOmega = Math.pow( MathUtil.applyDeadband(omega.getAsDouble(), DEADBAND), 3);

        Drive.DiffDrive(ProsY, ProOmega);
      }, Drive);
  }
}
