package frc.robot.subsystems.Intake.IntakeWheal;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeWhealsConSim implements IntakeWhealsConModlue {
    
    DCMotorSim motor = new DCMotorSim(
        /*new DCMotor(12,
         1,
         26,
         2,
         Units.rotationsToRadians(37 / 60),
         1)*/ DCMotor.getNEO(1),
         1, 10);

    public void setSpeed(double Speed){
        motor.setInputVoltage(Speed * RobotController.getInputVoltage());
        motor.update(0.02);
    }

    public double getVelcocity(){
        return motor.getAngularVelocityRPM();
    }
}
