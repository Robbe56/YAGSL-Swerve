// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangSubsystem extends SubsystemBase {
  /** Creates a new HangSubsystem. */
  VictorSP hangMotor;

  public HangSubsystem() {
    hangMotor = new VictorSP(Constants.Hanger.hangMotorID);
  }
  public void StopHangMotor(){
    hangMotor.stopMotor();
  }

  public void HangOnChain(){
    hangMotor.set(Constants.Hanger.HangSpeed);
  }

  public void Unwind(){
    hangMotor.set(Constants.Hanger.UnwindSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
