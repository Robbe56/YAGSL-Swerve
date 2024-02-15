// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoDumpInAmp extends Command {
  /** Creates a new AutoDumpInAmp. */
   private final ShooterSubsystem shooter;
   private final XboxController operatorController;
   private final Timer timer;

  public AutoDumpInAmp(ShooterSubsystem m_shooter, XboxController m_operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = m_shooter;
    operatorController = m_operatorController;
    timer = new Timer(); 
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.FeedMotorSlow();
    shooter.ShooterIntoAmpSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.StopFeedRoller();
    shooter.StopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > Constants.Shooter.TimeToRunShooterIntoAmp || !operatorController.getRawButton(1); //move arm down when timer stops or when button 1 stops being pressed
  }
}
