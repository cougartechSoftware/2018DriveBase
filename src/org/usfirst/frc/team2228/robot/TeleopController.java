package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleopController {
	private Joystick joystick;
	private SRXDriveBase driveBase;

	private short loggerIterations = 0;
	private short loggerThreshold = 20;
	
	private int autoCmdSequence = 1;
	private int timePeriodSF = TeleopControllerCfg.kHighSmoothPeriod;
	
	private double previousEMAValue = 0.0; // -1 to 1
	private static double test = 0;
	private double smoothFactor = 1.0;
	
	private boolean lastButtonRead = false, isButtonCmdActive = false;
	private boolean isButtonCmdActiveB = false;
	private boolean lastButtonReadB = false;
	
	private String lastMsgString = " ";

	public TeleopController(Joystick _joystick, SRXDriveBase _driveBase) {
		joystick = _joystick;
		driveBase = _driveBase;
	}
	public void teleopInit() {
		driveBase.setClearActionFlags();
	}
	public void teleopPeriodic() {
		double origThrottle = joystick.getRawAxis(DriverConfig.throttle);
		double origTurn = joystick.getRawAxis(DriverConfig.turn);

		double turn = origTurn;
		double throttle = origThrottle;
		throttle = AdjustForControllerDeadBand(throttle);
		turn = AdjustForControllerDeadBand(turn);
//		turn = CheckTurnSensitivityFilter(limit(turn));
//		throttle = CheckThrottleSensitivity(limit(throttle));
//		throttle = CheckSmoothMove(limit(throttle));

//		throttle = AdjustForControllerDeadBand(throttle);
//		turn = AdjustForControllerDeadBand(turn);
//		CheckForAdjustSpeedRequest();
//		driveBase.UpdateSRXDriveDataDisplay();
//		driveBase.setThrottleTurn((-throttle * .7), (turn * .7), false);
		getButtonA();
	    getButtonB();
	}	
//====================================================================
	private void getButtonA(){
		if (!joystick.getRawButton(XBoxConfig.A_BUTTON) && lastButtonRead) {
			isButtonCmdActive = true;
			msg("Botton A hit");
		} else if (isButtonCmdActive) {
			if (!driveBase.rotateToAngle(180, .3)) {
				isButtonCmdActive = false;
				msg("Botton A done");
			}
		}
		lastButtonRead = joystick.getRawButton(XBoxConfig.A_BUTTON); 
	}
	private void getButtonB(){
		if (!joystick.getRawButton(XBoxConfig.B_BUTTON) && lastButtonReadB) {
					isButtonCmdActiveB = true;	
					autoCmdSequence = 1;
		} else if (isButtonCmdActiveB) {
			switch(autoCmdSequence){
				case 1:
					// move 10 inches
					msg("case 1");
					if (!driveBase.velMoveToPosition(10, 0.2, true)) {
						autoCmdSequence = 2;
					};
					break;
				case 2:
					// turn right 90 deg
					msg("case 2");
					if(!driveBase.turnByEncoderToAngle(90, 25, .1, false, true )){
						autoCmdSequence = 3;
					};
					break;
				case 3:
					// move 10 in
					msg("case 3");
					if (!driveBase.velMoveToPosition(25, 0.2, true)) {
						autoCmdSequence = 4;
					};
					break;
				case 4:
					// turn left 90 deg
					msg("case 4");
					if(!driveBase.turnByEncoderToAngle(-90, 25, .1, false, false )){
						isButtonCmdActiveB = false;	
					};
					break;	
				default:
					isButtonCmdActiveB = false;	
			}
			
		}
		lastButtonReadB = joystick.getRawButton(XBoxConfig.B_BUTTON);
	}
	
	private void msg(String _msgString){
		if (_msgString != lastMsgString){
			System.out.println(_msgString);
			lastMsgString = _msgString;}
		}
	/********************
	 * Joystick Filtering Functions
	 ******************************/

	public double CheckTurnSensitivityFilter(double _turn) {
		/*
		 * Used for chessy turn, developed by team 254 for the turn stick to
		 * provide a more realistic feel for turning
		 */
		double fTurn = _turn;
		if (TeleopControllerCfg.isTurnSensitivityEnabled) {
			if (TeleopControllerCfg.isLowSpeedFactorEnabled) {
				fTurn = ApplySineFunction(fTurn);
				fTurn = ApplySineFunction(fTurn);
			} else {
				fTurn = ApplySineFunction(fTurn);
				fTurn = ApplySineFunction(fTurn);
				fTurn = ApplySineFunction(fTurn);
			}
		}
		return fTurn;
	}

	public double SineAdjustment(double _value) {
		double adjustedValue = _value;
		if (_value < 0) {
			adjustedValue = (2 * (-(Math.pow(_value, 3)))) - (3 * (Math.pow(_value, 2)));
		} else if (_value > 0) {
			adjustedValue = (3 * (Math.pow(_value, 2))) - (2 * (-(Math.pow(_value, 3))));
		}
		return adjustedValue;
	}

	public double CheckThrottleSensitivity(double _throttle) {
		/*
		 * Sensitivity modifies the input value to provide a different feel of
		 * robot motion to the operator. There are several sensitivity curves
		 * that adjust the operator input for driving the robot. - Linear
		 * sensitivity curve: The action is linear for the operator, output ==
		 * input - Sine wave sensitivity: The sensitivity provides a larger do
		 * little around zero speed and near full speed. This the typical
		 * elevator curve. - Squared sensitivity curve: This sensitivity slows
		 * down the response of the robot to fast moves on the part of the
		 * operator. - Cubed sensitivity curve: This sensitivity really slows
		 * down the response of the robot.
		 */
		double fThrottle = _throttle;

		switch (TeleopControllerCfg.sensitivitySet) {
		case Linear:
			// no change
			break;

		case Sine:
			fThrottle = SineAdjustment(_throttle);
			break;

		case Squared:
			fThrottle = (Math.pow(_throttle, 2));
			break;

		case Cubed:
			fThrottle = (TeleopControllerCfg.kThrottleCubedGain * (Math.pow(_throttle, 3)))
					+ ((1 - TeleopControllerCfg.kThrottleCubedGain) * _throttle);
			break;

		default:
			// complain about an unrecognized setting
			break;

		}
		return fThrottle;
	}

	/**
	 * TippingFilter aka SmoothMove The tipping filter follows the actions of
	 * the driver with respect to the motion of the throttle joystick. If the
	 * driver exceeds the limit of robot accel/decel capability, the tipping
	 * filter slows the response of the throttle to protect the robot. If the
	 * filter is activated, it will return to driver control as soon as the
	 * driver is controlling within robot limits. Determination of
	 * kMaxDeltaVelocity is determined by testing.
	 * 
	 * @param value,
	 *            is the value of the throttle joystick
	 */
	public double CheckSmoothMove(double _throttle) {
		double fThrottle = _throttle;
		double deltaValue = fThrottle - previousEMAValue;

		// if (driver.GetSmoothMoveEnabled()) {
		if ((fThrottle > 0) && (previousEMAValue < -TeleopControllerCfg.ZERO_DEAD_BAND)) // ||
																							// ((value
																							// <
																							// 0)
																							// &&
		// (oldEMA > 0))){
		{
			// we're tipping!!
			fThrottle = 0;
			timePeriodSF = TeleopControllerCfg.kHighSmoothPeriod;
			// System.out.println("Tipping forward");
		} else if ((fThrottle < 0) && (previousEMAValue > TeleopControllerCfg.ZERO_DEAD_BAND)) {// we're
																								// tipping!!
			fThrottle = 0;
			timePeriodSF = TeleopControllerCfg.kHighSmoothPeriod;
			// System.out.println("tipping backward");
		}

		double smoothFactor = 2.0 / (timePeriodSF + 1);
		fThrottle = previousEMAValue + smoothFactor * (fThrottle - previousEMAValue);

		if (Math.abs(previousEMAValue) < TeleopControllerCfg.ZERO_DEAD_BAND) {
			timePeriodSF = TeleopControllerCfg.kLowSmoothPeriod;
		}

		previousEMAValue = fThrottle;

		// SmartDashboard.putNumber("smooth", value);

		return fThrottle;
	}

	/**
	 * TippingFilter Team/Date/Author: The tipping filter follows the actions of
	 * the driver with respect to the motion of the throtle joystick. If the
	 * driver exceeds the limit of robot accel/decel capability the tipping
	 * filter slows the response of the throtle to protect the robot.
	 *
	 * There are four changes in value from the last joystick value: 1)
	 * Transistion from one side of zero to the other side of zero 2) The
	 * positive side of zero 3) The negative side of zero 4) Within the joystick
	 * deadband
	 *
	 * Determination of kMaxDeltaVel is determined by testing.
	 *
	 * @parm _value, is the value of the throtle joystick
	 */
	public double CheckTippingFilter(double _value) {
		double value = _value;
		// determine change for last joystick read
		double deltaValue = value - previousEMAValue;
		double timePeriodSF = 0.0;

		// Check joystick value transition from one side of zero to the other
		// side of zero
		if (Math.signum(value) != Math.signum(previousEMAValue)) {

			// If joystick change is large enough to cause a wheelie or cause
			// the
			// robot to start to tip - the robot intervenes to see that this
			// does
			// not occur The following limits the change in joystick movement
			if (Math.abs(deltaValue) > TeleopControllerCfg.kTransitionMaxDelta) {
				smoothFactor = TeleopControllerCfg.kTransitionSmoothFactor;
			} else {

				// If driver behaves
				smoothFactor = TeleopControllerCfg.klowSmoothFactor;
			}
		}

		// Determine if the sign of value and oldEMA are the same
		else if (Math.signum(value) == Math.signum(previousEMAValue)) {

			// Check for large deltaValue that may cause a wheelie or
			// rotation torque to a high Center of gravity on decel

			if (Math.abs(deltaValue) > TeleopControllerCfg.kMaxDeltaVelocity) {
				smoothFactor = TeleopControllerCfg.kHighSmoothFactor;
			} else {

				// If driver behaves
				smoothFactor = TeleopControllerCfg.klowSmoothFactor;
			}
		}

		// Check if the smoothing filter is within the joystick deadband and put
		// filter in high response gain
		if (Math.abs(value) < TeleopControllerCfg.ZERO_DEAD_BAND) {
			value = 0; // not previousValue?
			smoothFactor = TeleopControllerCfg.klowSmoothFactor;
		}
		// Run through smoothing filter
		/*
		 * Exponential Avg Filter (EMA) is a recursive low pass filter that can
		 * change it's gain to address filter response
		 * 
		 * Range of smoothFactor is 0 to 1; where smoothFactor = 0 (no
		 * smoothing) smoothFactor = .99999 high smoothing Typical smoothFactor
		 * = 1-(2.0 / (timePeriodSF + 1)) where user decides on aprox number of
		 * cycles for output = input. Time period on iterative robot class is
		 * aprox 20ms
		 */

		value = previousEMAValue + smoothFactor * (value - previousEMAValue);
		previousEMAValue = value;

		return value;
	}

	public double ApplySineFunction(double _turn) {
		// kTurnSensitivityHighGain should be 0.1 to 1.0 used for chezy turn
		// control
		double factor = Math.PI / 2.0 * TeleopControllerCfg.kTurnSensitivityHighGain;
		return Math.sin(factor * _turn) / Math.sin(factor);
	}

	public double AdjustForControllerDeadBand(double value) {
		if (Math.abs(value) < TeleopControllerCfg.ZERO_DEAD_BAND) {
			return 0;
		} else
			return value;

	}

	/*
	 * helper function to keep inside of acceptable %power range
	 */
	protected static double limit(double num) {
		if (num > 1.0) {
			return 1.0;
		}
		if (num < -1.0) {
			return -1.0;
		}
		return num;
	}

}
