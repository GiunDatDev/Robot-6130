/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;


/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also
 * range from -1 to 1 making it easy to work together.
 */
public class Robot extends TimedRobot {
  private static final int masterLeftPort = 0;
  private static final int slaveLeftPort = 1;
  private static final int masterRightPort = 2;
  private static final int slaveRightPort = 3;
  private static final int controllerPort = 0;
  
  // Define a deathzone for the joystick
  private static final double joystickDeathZone = 0.3;


  private SpeedController masterLeftMotor, slaveLeftMotor, masterRightMotor, slaveRightMotor;
  private Joystick driverController;

  @Override
  public void robotInit() {
    masterLeftMotor = new PWMVictorSPX(masterLeftPort);
    slaveLeftMotor = new PWMVictorSPX(slaveLeftPort);
    masterRightMotor = new PWMVictorSPX(masterRightPort);
    slaveRightMotor = new PWMVictorSPX(slaveRightPort);
    driverController = new Joystick(controllerPort);
  }

  // Running forward or backward depended on the vertical value of the joystick
  // Running left or right depended on the horizontal value of the joystick
  // Note: Check the direction of the robot to see if the robot runs in the right direction
  // If the robot turns left, the left side will run backward and the right side will run forward
  // If the robot turns right, the left side will run forward and the right side will run backward
  // Changing the direction of the robot to left or right need to be done first
  // Running forward and backward will be done after the robot changes it horizontal direction
  private void verticalDrive(double controllerVal){
    masterLeftMotor.set(-controllerVal);
    slaveLeftMotor.set(-controllerVal);
    masterRightMotor.set(controllerVal);
    slaveRightMotor.set(controllerVal);
  }

  private void horizontalDrive(double controllerVal){
    masterLeftMotor.set(controllerVal);
    slaveLeftMotor.set(controllerVal);
    masterRightMotor.set(controllerVal);
    slaveRightMotor.set(controllerVal);
  }

  private boolean checkJoystickZone(double controllerVal){
      if (controllerVal < -joystickDeathZone || controllerVal > joystickDeathZone){
        return true;
      }
      else {
        return false;
    }
  }

  // This function will check if the user want to change the horizontal direction of the robot or not
  // If it is true, the robot will change the direction first
  // Otherwise, the robot will continue to move in vertical direction
  private void driveRobot(){
    double controllerXValue = driverController.getX();
    double controllerYValue = driverController.getY();

   while (checkJoystickZone(controllerXValue) == true){
     horizontalDrive(controllerXValue);
   }
   if (checkJoystickZone(controllerYValue)){
     verticalDrive(controllerYValue);
   }
  }

  @Override
  public void teleopPeriodic() {
    driveRobot();
  }
}
