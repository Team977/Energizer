package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.subsystems.MotorConst;


public class AimSim {
    
    static final CANSparkMax AimMotor = new CANSparkMax(MotorConst.AimMotorCANId, MotorType.kBrushless);
    static final RelativeEncoder ENCODER = AimMotor.getEncoder();

    /** Creates a new Aim. */
    public AimSim() {}
  
    public void setSpeed(double speed){
      AimMotor.set(speed);
    }


}
