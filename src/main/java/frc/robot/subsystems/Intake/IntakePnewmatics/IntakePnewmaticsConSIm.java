package frc.robot.subsystems.Intake.IntakePnewmatics;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.subsystems.Intake.IntakeConst;

public class IntakePnewmaticsConSIm implements IntakePnewmaticsConMoudlue {
    
    boolean Out = false;

      static final DoubleSolenoidSim leftDoubleSolenoid 
    = new DoubleSolenoidSim(
      PneumaticsModuleType.REVPH,
      IntakeConst.LeftIntakeSOlChannle1, 
      IntakeConst.LeftIntakeSOlChannle2);

  static final DoubleSolenoidSim rightDoubleSolenoid 
    = new DoubleSolenoidSim(
      PneumaticsModuleType.REVPH,
      IntakeConst.RightIntakeSOlChannle1, 
      IntakeConst.RightIntakeSOlChannle2);

    public boolean Switch(){
        if (Out) {

            leftDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
            rightDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      
            Out = false;
          }else{
      
            leftDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
            rightDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
      
            Out = true;
          }

          return Out;
    }

}
