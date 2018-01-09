package org.usfirst.frc.team2228.robot;

public class TeleopControllerCfg {
	//  defines minimum joystick moves to be acted on 
	public static final double kInvertJoystick = -1.0;
	
    public static final double MAX_DELTA_VELOCITY = 0.2;
    public static final double ZERO_DEAD_BAND = 0.2;
    
    // used the in the sine function for turning
    public static boolean isTurnSensitivityEnabled = true;
    public static boolean isLowSpeedFactorEnabled = true;
    public static final double kTurnSensitivityHighGain = 0.7; // 0.1 to 1.0 used for chezy turn control
    
    public enum ThrottleSensitivity {
		Linear,
		Sine,
		Squared,
		Cubed
	}
    // used to vary the affect when using cubed sensitivity

    public static ThrottleSensitivity sensitivitySet = ThrottleSensitivity.Linear;
    public static final double kThrottleCubedGain = 1.0; // 0 to 1; 0 is linear (output==input) 1 is cubed (output=input**3)
    
    
    
	// used for "smooth move" -- the tipping filter for extreme changes in joystick
	public static final int kLowSmoothPeriod = 5;
	public static final int kHighSmoothPeriod = 35;
	// smoothFactor is 0 to 1, where 0 is no smooth, 0.99999 is high smoothing
	// Typical smooth factor = 2.0/ (timePeriodSF + 1) where user decides
	// on approx number of cycles for output == input.  Time period on 
	// iterative robot class is 20+ ms
	public static final double klowSmoothFactor = 0.3333;
	public static final double kHighSmoothFactor = 0.05556;
	public static final double kTransitionSmoothFactor = 0.05556;
	
	// determination of max delta values are determined by testing
	public static final double kMaxDeltaVelocity = 0.2;
	public static final double kTransitionMaxDelta = 0;

}
