package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CommandSubsystems.DriveSubsystem;

public class DriveStraightCommand extends CommandBase{
    private final DriveSubsystem m_DriveSubsystem;

  public DriveStraightCommand(DriveSubsystem subsystem) {
    m_DriveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_DriveSubsystem.arcadeDrive(.5, 0);
  }

  // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
