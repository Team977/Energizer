package frc.robot.subsystems.Shooter.Aim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public interface AimMotorConMoudle {
    public Rotation2d getRawRotation();
    public Measure<Velocity<Angle>> getVelcocity();
    public double getVolts();
    public double getCurrent();

    public void setSpeed(double speed);
    public void setVolts(double volts);

}
