// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class OI {
    XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER);
	XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER);
	
	Trigger dButtonA = new JoystickButton(this.driverController, Constants.BUTTON_A);
	Trigger dButtonB = new JoystickButton(this.driverController, Constants.BUTTON_B);
	Trigger dButtonX = new JoystickButton(this.driverController, Constants.BUTTON_X);
	Trigger dButtonY = new JoystickButton(this.driverController, Constants.BUTTON_Y);
	Trigger dButtonBack = new JoystickButton(this.driverController, Constants.BACK_BUTTON);
	Trigger dButtonStart = new JoystickButton(this.driverController, Constants.START_BUTTON);
	Trigger dButtonRightBumper = new JoystickButton(this.driverController, Constants.RIGHT_BUMPER);
	Trigger dButtonLeftBumper = new JoystickButton(this.driverController, Constants.LEFT_BUMPER);

    Trigger oButtonA = new JoystickButton(this.operatorController, Constants.BUTTON_A);
	Trigger oButtonB = new JoystickButton(this.operatorController, Constants.BUTTON_B);
	Trigger oButtonY = new JoystickButton(this.operatorController, Constants.BUTTON_Y);
	Trigger oButtonX = new JoystickButton(this.operatorController, Constants.BUTTON_X);
	Trigger oButtonBack = new JoystickButton(this.operatorController, Constants.BACK_BUTTON);
	Trigger oButtonStart = new JoystickButton(this.operatorController, Constants.START_BUTTON);
    Trigger oButtonRightStick = new JoystickButton(this.operatorController, Constants.RIGHT_STICK_BUTTON);
	Trigger oButtonRightBumper = new JoystickButton(this.operatorController, Constants.RIGHT_BUMPER);
	Trigger oButtonLeftBumper = new JoystickButton(this.operatorController, Constants.LEFT_BUMPER);
    Trigger oButtonRightTrigger = new JoystickButton(this.operatorController, Constants.RIGHT_TRIGGER);
	Trigger oButtonLeftTrigger = new JoystickButton(this.operatorController, Constants.LEFT_TRIGGER);

	POVButton oDPADUp = new POVButton(this.operatorController, Constants.DPAD_UP);
	POVButton oDPADDown = new POVButton(this.operatorController, Constants.DPAD_DOWN);
	POVButton oDPADLeft = new POVButton(this.operatorController, Constants.DPAD_LEFT);
	POVButton oDPADRight = new POVButton(this.operatorController, Constants.DPAD_RIGHT);

	POVButton dDPADUp = new POVButton(this.driverController, Constants.DPAD_UP);
	POVButton dDPADDown = new POVButton(this.driverController, Constants.DPAD_DOWN);
	POVButton dDPADLeft = new POVButton(this.driverController, Constants.DPAD_LEFT);
	POVButton dDPADRight = new POVButton(this.driverController, Constants.DPAD_RIGHT);




  	public boolean getOperatorButton(int axis) {
		return this.operatorController.getRawButton(axis);
	}
	
	public boolean getDriverButton(int axis) {
		return this.driverController.getRawButton(axis);
	}
	
	public double getOperatorRawAxis(int axis) {
		return this.operatorController.getRawAxis(axis);
	}
	
	public double getDriverRawAxis(int axis) {
		return this.driverController.getRawAxis(axis);
	}
	
	public int getOperatorPOV(){
		return this.operatorController.getPOV();
	}

	public int getDriverPOV(){
		return this.driverController.getPOV();
	}
}
