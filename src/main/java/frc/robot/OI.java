/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {

	// Creates private joysticks objects for use.
	private static final Joystick leftController = new Joystick(1);
	// Enum that represents all possible buttons in use.
	
	// Public method that returns the left joystick's deadzone-adjusted values
	public double getLeftJoystick(final char input)
	{
		switch(input)
		{
			case 'X': // Gets deadzone corrected x-axis position
				return (leftController.getRawAxis(0));
			case 'Y': // Gets deadzone corrected y-axis position
				return (leftController.getRawAxis(1));
			case 'Z': // Gets deadzone corrected z-axis (rotation) position
				return 0.0;
			default: // Returns 0.0 is argument is unknown
				return 0.0;
		}
	}
	// Public method that returns the right joystick's deadzone-adjusted values
	public double getRightJoystick(final char input)
	{
		switch(input)
		{
			case 'X': // Gets deadzone corrected x-axis position
				return (leftController.getRawAxis(4));
			case 'Y': // Gets deadzone corrected y-axis position
				return -(leftController.getRawAxis(5));
			case 'Z': // Gets deadzone corrected z-axis (rotation) position
				return 0.0;
			default: // Returns 0.0 is argument is unknown
				return 0.0;
		}
	}
}