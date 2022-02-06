package frc.robot;

import frc.robot.Constants.ControllerConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.DriveStraightButtonCommand;
import frc.robot.subsystems.base.DriveSubsystem;

public class OI {

	// Gamepads
	public static final XboxController driverGamepad = new XboxController(ControllerConstants.kDriverControllerPort); // Driver
	public static final XboxController operatorGamepad = new XboxController(ControllerConstants.kOperatorControllerPort);

	/**
	 * Method that returns if the joystick is within the deadband restraing
	 * @param val Value of the joystick
	 * @param deadband Allotted Deadband
	 * @return Either the value if outside deadband or 0.0 if inside deadband
	 */
	private static double deadBand(double val, double deadband) {
		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}

	public static double getDriverLeftX() {
		return deadBand(driverGamepad.getLeftX(), ControllerConstants.kDriverDeadBandLeftX);
	}

	public static double getDriverRightX() {
		return deadBand(driverGamepad.getLeftX(), ControllerConstants.kDriverDeadBandRightX);
		// return deadBand(driverGamepad.getRightX(), ControllerConstants.kDriverDeadBandRightX);
	}

	public static double getDriverLeftY() {
		return deadBand(driverGamepad.getLeftY(), ControllerConstants.kDriverDeadBandLeftY);
	}

	public static double getDriverRightY() {
		return deadBand(driverGamepad.getLeftY(), ControllerConstants.kDriverDeadBandRightY);
		// return deadBand(driverGamepad.getRigtY(), ControllerConstants.kDriverDeadBandRightY);
	}

	public static double getOperatorLeftX() {
		return deadBand(operatorGamepad.getLeftX(), ControllerConstants.kDriverDeadBandLeftX);
	}

	public static double getOperatorRightX() {
		return deadBand(operatorGamepad.getLeftX(), ControllerConstants.kDriverDeadBandRightX);
		// return deadBand(driverGamepad.getRightX(), ControllerConstants.kDriverDeadBandRightX);
	}

	public static double getOperatorLeftY() {
		return deadBand(operatorGamepad.getLeftY(), ControllerConstants.kDriverDeadBandLeftY);
	}

	public static double getOperatorRightY() {
		return deadBand(operatorGamepad.getLeftY(), ControllerConstants.kDriverDeadBandRightY);
		// return deadBand(driverGamepad.getRigtY(), ControllerConstants.kDriverDeadBandRightY);
	}

	public static void configureButtonBindings(DriveSubsystem m_robotDrive) {
		// When Driver.A is pressed, run new DriveStraightButtonCommand
		new JoystickButton(driverGamepad,Button.kA.value)
			.whenPressed(new DriveStraightButtonCommand(m_robotDrive));

		//When Driver.B is pressed, run new DriveStraightButtonCommand
		new JoystickButton(driverGamepad, Button.kB.value)
			.whenPressed(new DriveStraightButtonCommand(m_robotDrive));

		//When Driver.RightBumper is pressed, shift into high gear
		//When Driver.RightBumper is released, shift into low gear
		new JoystickButton(driverGamepad, Button.kRightBumper.value)
			.whenPressed(() -> m_robotDrive.shiftGearHigh())
			.whenReleased(() -> m_robotDrive.shiftGearLow());

		//When Driver.LeftBumper is pressed, drop omni wheels
		//When Driver.LeftBumper is released, raise omni wheels
		new JoystickButton(driverGamepad, Button.kLeftBumper.value)
			.whenPressed(() -> m_robotDrive.dropWheels())
			.whenReleased(() -> m_robotDrive.liftWheels());

	}
}