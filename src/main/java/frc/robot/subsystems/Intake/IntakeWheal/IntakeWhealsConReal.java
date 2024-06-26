package frc.robot.subsystems.Intake.IntakeWheal;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import frc.robot.subsystems.Intake.IntakeConst;

public class IntakeWhealsConReal implements IntakeWhealsConModlue {
    
      static final PWMVictorSPX IntakeMotor 
    = new PWMVictorSPX(IntakeConst.IntakeWhealChannle);

    public void setSpeed(double Speed){
        IntakeMotor.set(Speed);
    }

    public double getVelcocity(){
        return 0;
    }

}
