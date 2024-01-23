// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeControl extends Command {
  /** Creates a new IntakeControl. */
  private final Intake intake;
  private final XboxController controller;
  public IntakeControl(Intake i, XboxController c) {
    intake = i;
    controller = c;
    addRequirements(intake);
   // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  if(controller.getRawButton(Constants.Intake.SpinButton))
  {
    intake.intakeActive();
  }
  else{
    intake.intakeRest();
  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
