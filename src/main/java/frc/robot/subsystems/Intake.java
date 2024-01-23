// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
WPI_TalonSRX topMotor;
WPI_TalonSRX bottomMotor;


  public Intake() {
  topMotor = new WPI_TalonSRX(Constants.Intake.topMotorCANID);
  bottomMotor = new WPI_TalonSRX(Constants.Intake.bottomMotorCANID);
  }
public void intakeRest()
{
  topMotor.stopMotor();
  bottomMotor.stopMotor();
}
public void intakeActive()
{
  topMotor.set(Constants.Intake.topMotorIntakeSpeed);
  bottomMotor.set(Constants.Intake.bottomMotorIntakeSpeed);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
