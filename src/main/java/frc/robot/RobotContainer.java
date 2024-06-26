// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Intake;
import frc.robot.commands.SpinupShooterMotor;
import frc.robot.commands.driveCommand;
import frc.robot.commands.setAimPosition;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Intake.IntakePnewmatics.IntakePnewmatics;
import frc.robot.subsystems.Intake.IntakeWheal.IntakeWheals;
import frc.robot.subsystems.Shooter.Aim.Aim;
import frc.robot.subsystems.Shooter.Shooter.Shooter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
  //    new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Joystick joystickSim = new Joystick(0);

  private final Aim aim = new Aim();
  private final Shooter shooter = new Shooter();
  private final IntakeWheals intakeWheals = new IntakeWheals();
  private final IntakePnewmatics intakePnewmatics = new IntakePnewmatics();
  private final Drive drive = new Drive();

  //private final drive drive = new drive();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    SmartDashboard.putNumber("set Aim Target Position", 20);
    SmartDashboard.putNumber("set Shooter RPS", 50);
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    aim.setDefaultCommand(new setAimPosition(() -> SmartDashboard.getNumber("set Aim Target Position", 0), aim));
    shooter.setDefaultCommand(new SpinupShooterMotor(() -> SmartDashboard.getNumber("set Shooter RPS", 0), shooter));
    intakeWheals.setDefaultCommand(new Intake(0.5, 0.5, 0.5, intakePnewmatics, intakeWheals));
    drive.setDefaultCommand( driveCommand.driveCommands(() -> joystickSim.getRawAxis(1), () -> joystickSim.getRawAxis(0), drive));
    //drive.setDefaultCommand
    //  (driveCommand.driveCommands(() -> joystickSim.getRawAxis(1), 
    //   () -> joystickSim.getRawAxis(0), drive));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
