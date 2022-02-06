package frc.robot.commands.drivetrain;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.DriveStraightCommand;
import frc.robot.subsystems.base.DriveSubsystem;

public class DriveStraightButtonCommand extends SequentialCommandGroup{
    public final DriveSubsystem m_DriveSubsystem; // i had this as private

  public DriveStraightButtonCommand(DriveSubsystem subsystem) {
    m_DriveSubsystem = subsystem;



    addCommands(
      new WaitCommand(2), 
          new DriveStraightCommand(m_DriveSubsystem).withTimeout(5),
          new WaitCommand(2),
          // new DriveStraightCommand(m_DriveSubsystem).withTimeout(2),

          // this race condition will complete when the 1st to complete finishes - so the robot will drive for 2 seconds
          race(
              new DriveStraightCommand(m_DriveSubsystem).withTimeout(5),
              new WaitCommand(2)
          )

        
    );
 
  }  
}

// example from wpilib using race:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html#inline-command-definitions
//  optional calls: sequence(), parallel(), race(), and deadline()

// public ExampleSequence() {
//   addCommands(
//       new FooCommand(),
//       race(
//           new BarCommand(),
//           new BazCommand()
//       )
//   );
// }