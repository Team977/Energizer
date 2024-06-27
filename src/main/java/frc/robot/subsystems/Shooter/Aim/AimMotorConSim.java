package frc.robot.subsystems.Shooter.Aim;

import static edu.wpi.first.units.Units.Rotations;
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

public class AimMotorConSim implements AimMotorConMoudle {
    
    static final DCMotorSim AimMotor = new DCMotorSim(DCMotor.getNEO(1), 6, 20);
    static final EncoderSim ENCODER = new EncoderSim( new Encoder(0, 1) );

    @Override
    public Rotation2d getRawRotation(){
        return new Rotation2d(Rotations.of(AimMotor.getAngularPositionRotations()));
    }

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
        AimMotor.setInputVoltage(speed * RobotController.getInputVoltage());
        AimMotor.update(0.02);
    }
    
    public void setVolts(double volts){
        AimMotor.setInputVoltage(volts);
    }

}
