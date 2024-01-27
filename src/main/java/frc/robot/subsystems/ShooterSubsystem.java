// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

WPI_TalonSRX armMotor;
WPI_TalonSRX feedMotor;
WPI_TalonSRX rightShooterMotor;
WPI_TalonSRX leftShooterMotor;

DigitalInput armAtRest;
DigitalInput armAtAmp;
DigitalInput noteInFeeder;

  public ShooterSubsystem() {

armMotor = new WPI_TalonSRX(Constants.Shooter.armMotorCANID);
feedMotor = new WPI_TalonSRX(Constants.Shooter.feedMotorCANID);
rightShooterMotor = new WPI_TalonSRX(Constants.Shooter.rightShooterMotorID);
leftShooterMotor = new WPI_TalonSRX(Constants.Shooter.leftShooterMotorID);



armAtRest = new DigitalInput(Constants.Shooter.armDownLimitSwitch);
armAtAmp = new DigitalInput(Constants.Shooter.armUpLimitSwitch);
noteInFeeder = new DigitalInput(Constants.Shooter.noteInFeederSensor);
}

  public void StopArm(){
    armMotor.stopMotor();
  }

  public void StopFeedRoller(){
    feedMotor.stopMotor();
  }

  public void StopShooter(){
    rightShooterMotor.stopMotor();
    leftShooterMotor.stopMotor();
  }

  public void ArmControl(double armCommandSpeed, double armPositionTarget){
    SmartDashboard.putNumber("Arm Encoder Value", armMotor.getSelectedSensorPosition());
    if (armAtRest.get() != false && armAtAmp.get() != false && armCommandSpeed < 0.05 && armCommandSpeed > -0.05) { //not on limit switch and not pressing joystick
      armPositionTarget = armMotor.getSelectedSensorPosition();
      armMotor.set(ControlMode.Position, armPositionTarget);
    }

    if(armCommandSpeed > 0 && armAtRest.get() == true){
      armMotor.set(ControlMode.PercentOutput, armCommandSpeed);
    }
    if (armCommandSpeed < 0 && armAtAmp.get() == true){
      armMotor.set(ControlMode.PercentOutput, armCommandSpeed);
    }
    if ((armCommandSpeed > 0 && armAtRest.get() == false) || (armCommandSpeed < 0 && armAtAmp.get() == false)){
      armMotor.stopMotor();
    }

  
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run


  }
}
