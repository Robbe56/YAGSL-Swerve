// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoFireFarCommand extends Command {
  /** Creates a new AutoFireCommand. */
  private final ArmSubsystem arm;
  private final ShooterSubsystem shooter;
  private final Timer timer;

  public AutoFireFarCommand(ArmSubsystem m_arm, ShooterSubsystem m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = m_arm;
    shooter = m_shooter;
    timer = new Timer();

    addRequirements(arm, shooter);
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
        if (arm.GetArmEncoderPosition() < Constants.Shooter.aimedFromSafeZone){
      arm.ArmUpCommand();
    }
    else {arm.ArmHoldPosition();}
  

  if (timer.get() < Constants.Shooter.SpinUpTime){
    shooter.StopFeedRoller();
    shooter.ShooterIntoSpeakerSpeed();
  }

  if (timer.get() > Constants.Shooter.SpinUpTime && timer.get() < Constants.Shooter.NoteInAirTime){
    shooter.FeedMotorFast();
    shooter.ShooterIntoSpeakerSpeed();
  }

  if (timer.get() > Constants.Shooter.NoteInAirTime){
    shooter.StopFeedRoller();
    shooter.StopShooter();
  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.StopFeedRoller();
    shooter.StopShooter();
    arm.StopArm();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > Constants.Shooter.NoteInAirTime;
  }
}
