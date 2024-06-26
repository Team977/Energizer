package frc.robot.subsystems.Shooter.Shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.robot.subsystems.MotorConst;

public class ShooterConReal implements ShooterConModlue {
    

    static final CANSparkMax AimMotor = new CANSparkMax(MotorConst.ShooterMotorCANId, MotorType.kBrushless);
    static final RelativeEncoder ENCODER = AimMotor.getEncoder();

    @Override
    public Measure<Velocity<Angle>> getVelcocity(){
        return RotationsPerSecond.of(ENCODER.getVelocity()/60);
    }


    @Override
    public double getVolts(){
        return AimMotor.getAppliedOutput() * AimMotor.getBusVoltage();
    }

    public double getCurrent(){
        return AimMotor.getOutputCurrent();
    }

    public void setSpeed(double speed){
        AimMotor.set(speed);
    }
    
    public void setVolts(double volts){
        AimMotor.setVoltage(volts);
    }

}
