// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangSubsystem;

public class HangerCommand extends Command {
  /** Creates a new HangerCommand. */
    private final HangSubsystem hanger;
  private final XboxController armController;
  private final XboxController driverController;

  public HangerCommand(HangSubsystem m_hanger, XboxController m_armController, XboxController m_driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    hanger = m_hanger;
    armController = m_armController;
    driverController = m_driverController;

    addRequirements(hanger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (armController.getRawButton(10) == false && driverController.getRawButton(8) == false){ // not pushing either "start button"
    hanger.StopHangMotor();
    }

    if (armController.getRawButton(10) == true && driverController.getRawButton(8) == false){ //only operator push start
      hanger.HangOnChain();
    }

    if (armController.getRawButton(10) == true && driverController.getRawButton(8) == true){ //both push start
    hanger.Unwind();
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
