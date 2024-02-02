// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  public ShooterSubsystem() {

armMotor = new WPI_TalonSRX(Constants.Shooter.armMotorCANID);
feedMotor = new WPI_VictorSPX(Constants.Shooter.feedMotorCANID);
rightShooterMotor = new WPI_VictorSPX(Constants.Shooter.rightShooterMotorID);
leftShooterMotor = new WPI_VictorSPX(Constants.Shooter.leftShooterMotorID);

armAtRest = new DigitalInput(Constants.Shooter.armDownLimitSwitch);
armAtAmp = new DigitalInput(Constants.Shooter.armUpLimitSwitch);
noteInFeeder = new DigitalInput(Constants.Shooter.noteInFeederSensor);

armMotor.configFactoryDefault();
feedMotor.configFactoryDefault();
rightShooterMotor.configFactoryDefault();
leftShooterMotor.configFactoryDefault();

armMotor.setSelectedSensorPosition(0);

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
   
    if (armAtRest.get() && armAtAmp.get() && armCommandSpeed < 0.05 && armCommandSpeed > -0.05) { //not on limit switch and not pressing joystick
      armMotor.set(Constants.Shooter.armHoldSpeed);
    }

    if(armCommandSpeed < 0 && armAtRest.get() == true){ //pulling back on joystick and not pressing down limit switch
      if (armCommandSpeed < Constants.Shooter.MaxDownSpeed){
        armMotor.set(Constants.Shooter.MaxDownSpeed);
        }
        
      else {armMotor.set(armCommandSpeed);}
    }

    if (armCommandSpeed > 0 && armAtAmp.get() == true){ //pushing forward on joystick and not pressing top limit switch
      if (armCommandSpeed > Constants.Shooter.MaxUpSpeed){
        armMotor.set(Constants.Shooter.MaxUpSpeed);
        }
      else {armMotor.set(armCommandSpeed);}
    }

    if ((armCommandSpeed < 0 && armAtRest.get() == false) || (armCommandSpeed > 0 && armAtAmp.get() == false)){
      armMotor.stopMotor();
    }
    if (armAtRest.get() == false){                      //pushing lower limit switch
      armMotor.setSelectedSensorPosition(0); //reset encoder
    }
   }

   public void ArmUpCommand(){
    if (armAtAmp.get() == false){ //if pressing top limit switch
      armMotor.stopMotor();
    }
    if (-armMotor.getSelectedSensorPosition()/10000 < Constants.Shooter.almostUpValue){ 
      armMotor.set(Constants.Shooter.armUpSpeed);
    }
    if (-armMotor.getSelectedSensorPosition()/10000 >= Constants.Shooter.almostUpValue){
      armMotor.set(Constants.Shooter.armUpSpeed*0.2);
    }  
    
   }

   public void ArmDownCommand(){
    if (armAtRest.get() == false){ //if pressing bottom limit switch
      armMotor.stopMotor();
    }
    if (-armMotor.getSelectedSensorPosition()/10000 > Constants.Shooter.almostDownValue){ 
      armMotor.set(Constants.Shooter.armDownSpeed);
    }
    if (-armMotor.getSelectedSensorPosition()/10000 <= Constants.Shooter.almostDownValue){
      armMotor.set(Constants.Shooter.armDownSpeed*0.15);
    } 
  }

   public void FeedMotorSlow(){
    feedMotor.set(Constants.Shooter.feedLowSpeed);
   }

    public void FeedMotorFast(){
    feedMotor.set(Constants.Shooter.feedHighSpeed);
   }

   public void ShooterIntoAmpSpeed(){
    rightShooterMotor.set(Constants.Shooter.shootLowSpeed);
    leftShooterMotor.set(Constants.Shooter.shootLowSpeed);
   }

    public void ShooterIntoSpeakerSpeed(){
    rightShooterMotor.set(Constants.Shooter.shootHighSpeed);
    leftShooterMotor.set(Constants.Shooter.shootHighSpeed);
   }

   public double GetArmEncoderPosition(){
    return -armMotor.getSelectedSensorPosition()/10000;
   }

   public boolean GetTopLimitSwitch(){
    return armAtAmp.get();
   }
    
    public boolean GetBottomLimitSwitch(){
    return armAtRest.get();
   }

   public void ResetArmEncoder(){
    if (armAtRest.get() == false){
      armMotor.setSelectedSensorPosition(0); //reset encoder
    }
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  SmartDashboard.putNumber("Arm Encoder Value", -armMotor.getSelectedSensorPosition()/10000);
  SmartDashboard.putBoolean("Low Arm Limit Switch", armAtRest.get());
  SmartDashboard.putBoolean("Top Arm Limit Switch", armAtAmp.get());

  }
}
