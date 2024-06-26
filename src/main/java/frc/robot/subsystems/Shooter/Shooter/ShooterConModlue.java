package frc.robot.subsystems.Shooter.Shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public interface ShooterConModlue {
    
    public Measure<Velocity<Angle>> getVelcocity();

    public double getVolts();
    public double getCurrent();

    public void setVolts(double Volts);
    public void setSpeed(double speed);

}
