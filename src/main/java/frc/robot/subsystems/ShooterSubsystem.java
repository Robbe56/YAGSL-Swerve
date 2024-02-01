// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

WPI_TalonSRX armMotor;
WPI_VictorSPX feedMotor;
WPI_VictorSPX rightShooterMotor;
WPI_VictorSPX leftShooterMotor;

DigitalInput armAtRest;
DigitalInput armAtAmp;
public DigitalInput noteInFeeder;

double armCommandSpeed;
double armEnconderValue;

  public ShooterSubsystem() {

armMotor = new WPI_TalonSRX(Constants.Shooter.armMotorCANID);
feedMotor = new WPI_VictorSPX(Constants.Shooter.feedMotorCANID);
rightShooterMotor = new WPI_VictorSPX(Constants.Shooter.rightShooterMotorID);
leftShooterMotor = new WPI_VictorSPX(Constants.Shooter.leftShooterMotorID);

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

  public void ArmJoystickControl(double armCommandSpeed){
   
    armEnconderValue = armMotor.getSelectedSensorPosition();

    if (armAtRest.get() != true && armAtAmp.get() != true && armCommandSpeed < 0.05 && armCommandSpeed > -0.05) { //not on limit switch and not pressing joystick
      armMotor.set(ControlMode.Position, armEnconderValue);
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

   public void ArmUpCommand(double armUpSpeed){
    if (armAtAmp.get() == false){ //if pressing top limit switch
      armMotor.stopMotor();
    }
    else{armMotor.set(ControlMode.PercentOutput, armUpSpeed);
   }}

   public void ArmDownCommand(double armDownSpeed){
    if (armAtRest.get() == false){ //if pressing bottom limit switch
      armMotor.stopMotor();
    }
    else{armMotor.set(ControlMode.PercentOutput, armDownSpeed);
   }}

   public void FeedMotorSlow(){
    feedMotor.set(Constants.Shooter.feedLowSpeed);
   }

    public void FeedMotorFast(){
    feedMotor.set(Constants.Shooter.feedHighSpeed);
   }

   public void ShooterIntoAmpSpeed(){
    rightShooterMotor.set(Constants.Shooter.shootLowSpeed);
    leftShooterMotor.set(-Constants.Shooter.shootLowSpeed);
   }

    public void ShooterIntoSpeakerSpeed(){
    rightShooterMotor.set(Constants.Shooter.shootHighSpeed);
    leftShooterMotor.set(-Constants.Shooter.shootHighSpeed);
   }

   public double GetArmEncoderPosition(){
    return armMotor.getSelectedSensorPosition();
   }

   public boolean GetTopLimitSwitch(){
    return armAtAmp.get();
   }
    
    public boolean GetBottomLimitSwitch(){
    return armAtRest.get();
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  SmartDashboard.putNumber("Arm Encoder Value", armMotor.getSelectedSensorPosition());

  }
}
