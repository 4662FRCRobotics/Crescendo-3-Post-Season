// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Libraries.ConsoleTele;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootNote extends Command {
  /** Creates a new ShootNote. */
/* */

  private Shooter m_shooter;
  private Intake m_Intake;
private Timer m_timer = new Timer();
private ConsoleTele m_Tele = new ConsoleTele(2);
  public ShootNote(Shooter shooter, Intake intake) {
m_shooter = shooter;
m_Intake = intake;
    addRequirements(shooter, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_shooter.Shoot(.9);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.Shoot(.9);
    if(m_timer.hasElapsed(ShooterConstants.kSPIN_UP_TIME)){
m_Intake.dropNote();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.Stop();
    m_Intake.holdNote();
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(ShooterConstants.kSPIN_UP_TIME + .25);
  }
}
