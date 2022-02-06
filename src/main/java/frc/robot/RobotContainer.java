package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FMSConstants;
import frc.robot.commands.auto.DriveStraightForTimeCommand;
import frc.robot.commands.auto.doNothingCommand;
import frc.robot.subsystems.base.Climber;
import frc.robot.subsystems.base.DriveSubsystem;
import frc.robot.subsystems.base.Intake;
import frc.robot.subsystems.base.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
// @SuppressWarnings({"unused"})
public class RobotContainer {
        /**
         * Drive Subsystem object
         */
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();

        private final Shooter m_shooter = new Shooter();

        private final Intake m_intake = new Intake();

        private final Climber m_climber = new Climber();

        /**
         * Trajectory Manager object
         */
        private final TrajectoryManager m_trajectoryManager = new TrajectoryManager();

        // Autonomous Option
        public static final SendableChooser<Command> m_chooser = new SendableChooser<>();

        // Alliance Information
        public static DriverStation.Alliance alliance;
        public static int allianceColor;
        private NetworkTableEntry allianceColorEntry;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Set the scheduler to log Shuffleboard events for command initialize,
                // interrupt, finish

                CommandScheduler.getInstance()
                                .onCommandInitialize(
                                                command -> Shuffleboard.addEventMarker(
                                                                "Command initialized", 
                                                                command.getName(),
                                                                EventImportance.kNormal));
                CommandScheduler.getInstance()
                                .onCommandInterrupt(
                                                command -> Shuffleboard.addEventMarker(
                                                                "Command interrupted", 
                                                                command.getName(),
                                                                EventImportance.kNormal));
                CommandScheduler.getInstance()
                                .onCommandFinish(
                                                command -> Shuffleboard.addEventMarker(
                                                                "Command finished", 
                                                                command.getName(),
                                                                EventImportance.kNormal));

                // Configure the button bindings
                OI.configureButtonBindings(m_robotDrive);

                // Configure default commands
                // Set the default drive command to split-stick arcade drive
                m_robotDrive.setDefaultCommand(
                                // A split-stick arcade command, with forward/backward controlled by the left
                                // hand, and turning controlled by the right.
                                new RunCommand(
                                                () -> m_robotDrive.arcadeDrive(
                                                                -OI.getDriverLeftY(), OI.getDriverRightX()),
                                                m_robotDrive));

                // Add options to the Autonomous widget
                m_chooser.addOption("s - curve", Ramsete(m_trajectoryManager.makeSTrajectory()));
                m_chooser.addOption("drive straight", Ramsete(m_trajectoryManager.driveStraightTrajectory()));
                m_chooser.setDefaultOption("do nothing", new doNothingCommand());
                m_chooser.setDefaultOption("drive for time", new DriveStraightForTimeCommand(m_robotDrive));

                // Put the chooser on the dashboard
                Shuffleboard.getTab("Autonomous").add(m_chooser);

                // Example of how to add custom shuffleboard widgets to the dashboard
                // once the widgets are defined and the network table entry is defined
                // update the value in any periodic code
                ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
                allianceColorEntry = tab.add("Alliance Color", -100)
                                .withPosition(2, 1)
                                .withSize(4, 2)
                                .getEntry();

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // this is where we run the initialization -see instantcommand: executes a
                // single action on initialization, and then ends immediately (on page
                // convenience features)
                // or do we put it in the autonomousInit in robot.java?
                return m_chooser.getSelected();
        }

        public Command Ramsete(Trajectory exampleTrajectory) {

                RamseteCommand ramseteCommand = new RamseteCommand(
                                exampleTrajectory,
                                m_robotDrive::getPose,
                                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                                new SimpleMotorFeedforward(
                                                DriveConstants.ksVolts,
                                                DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                DriveConstants.kDriveKinematics,
                                m_robotDrive::getWheelSpeeds,
                                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                                // RamseteCommand passes volts to the callback
                                m_robotDrive::tankDriveVolts,
                                m_robotDrive);

                // Reset odometry to the starting pose of the trajectory.
                m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

                // Run path following command, then stop at the end.
                return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
        }

        // findColor - sets the global variable allianceColor based on the color of the
        // alliance coming from the fmsdata
        public void findAllianceColor() {
                try {
                        // robot is ENABLED
                        if (RobotController.isSysActive()) {
                                // connected to FMS
                                allianceColor = FMSConstants.ALLIANCE_INITIALIZED; // 0
                                alliance = DriverStation.getAlliance();

                                String gameData = DriverStation.getGameSpecificMessage();
                                SmartDashboard.putString("gameData", gameData);

                                if (alliance == Alliance.Blue) {
                                        allianceColor = FMSConstants.ALLIANCE_BLUE; // -1
                                } else if (alliance == Alliance.Red) {
                                        allianceColor = FMSConstants.ALLIANCE_RED; // 1
                                } else {
                                        allianceColor = FMSConstants.ALLIANCE_INITIALIZED; // 0
                                }
                        } else { // robot is NOT ENABLED
                                allianceColor = FMSConstants.ALLIANCE_NOT_ENABLED; // 20
                        }

                } catch (Exception e) {
                        allianceColor = FMSConstants.ALLIANCE_EXCEPTION; // 11
                }
                // example on updating the dashboard using the smart dashboard implementation
                SmartDashboard.putNumber("Alliance", allianceColor);

                // exmaple on updating the dashboard using the Shuffleboard custom widget and
                // the network table entry
                allianceColorEntry.setDouble(allianceColor);

        }

}
