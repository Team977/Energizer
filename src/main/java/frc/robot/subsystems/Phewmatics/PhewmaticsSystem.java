// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Phewmatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhewmaticsSystem extends SubsystemBase {
  /** Creates a new PhewmaticsSystem. */


  private final Compressor compressor 
    = new Compressor(PneumaticsModuleType.REVPH);

  public PhewmaticsSystem() {}

  @Override
  public void periodic() {

    if(PhewmaticsConst.Anglog){
    compressor.enableAnalog(PhewmaticsConst.minPSI, PhewmaticsConst.maxPSI);
    }else{
      compressor.enableHybrid(PhewmaticsConst.minPSI, PhewmaticsConst.maxPSI);
    }
    // This method will be called once per scheduler run
  }

  public double getVoltage(){
    return compressor.getAnalogVoltage();
  }

  public boolean getPressureSwitch(){
    return compressor.getPressureSwitchValue();
  }

  public double getCurrent(){
    return compressor.getCurrent();
  }

  public double getPresure(){
    return compressor.getPressure();
  }

  public void disable(){

    compressor.disable();
  }
}
