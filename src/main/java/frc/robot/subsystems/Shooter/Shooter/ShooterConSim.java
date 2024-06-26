package frc.robot.subsystems.Shooter.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

/**
 * ShooterConSim
 */
public class ShooterConSim implements ShooterConModlue {

    static final DCMotorSim AimMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 2);

    @Override
    public Measure<Velocity<Angle>> getVelcocity(){
        return RotationsPerSecond.of(AimMotor.getAngularVelocityRPM()/60);
    }

    @Override
    public double getVolts(){
        return 0;
    }

    public double getCurrent(){
        return AimMotor.getCurrentDrawAmps();
    }

    public void setSpeed(double speed){
        AimMotor.setInputVoltage(speed * RobotController.getInputVoltage());;
        AimMotor.update(0.02);
    }
    
    public void setVolts(double volts){
        AimMotor.setInputVoltage(volts);
    }
    
}