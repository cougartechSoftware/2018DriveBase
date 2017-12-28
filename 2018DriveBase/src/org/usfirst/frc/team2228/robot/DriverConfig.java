package org.usfirst.frc.team2228.robot;

public class DriverConfig {
    
	// this is a good place for all your button mapping
    // controller calls getRawAxis with this id for analog input (joysticks and triggers)
    // so call looks like joystick.getRawAxis(DriverConfig.throttle) and never changes
	// 
	public static int throttle = XBoxConfig.LEFT_STICK_Y_AXIS;
    public static int turn = XBoxConfig.RIGHT_STICK_X_AXIS;
    
    // controller calls getRawButton with id for button map
    // use defines for functions you need so calling code never changes
    // joystick.getRawButton(DriverConfig.whatever) and map whatever to xbox button
    public static int slowSpeedEnable = XBoxConfig.LEFT_TRIGGER;
    public static int highSpeedEnable = XBoxConfig.RIGHT_TRIGGER;

}
