// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ArmDownAutoCommand extends Command {
  /** Creates a new ArmDownAutoCommand. */
  private final ShooterSubsystem shooter;

  public ArmDownAutoCommand(ShooterSubsystem m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = m_shooter; 
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.GetArmEncoderPosition() >= Constants.Shooter.almostDownValue){
    shooter.ArmDownCommand(Constants.Shooter.armDownSpeed);
    }
    if (shooter.GetArmEncoderPosition() < Constants.Shooter.almostDownValue){
      shooter.ArmDownCommand(Constants.Shooter.armDownSpeed*.1);                //go at 10% of arm speed when close to limit
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.StopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !shooter.GetBottomLimitSwitch(); //returns true when bottom limit switch is pressed (limit switch goes from true to false with "!")
}
}
