/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


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
  private static final int compressorPort = 0;

  // Define a zone to activate pneumatic
  private static final double joystickDeathZone = 0.3;


  private SpeedController masterLeftMotor, slaveLeftMotor, masterRightMotor, slaveRightMotor;
  private XboxController driverController;
  private DifferentialDrive driveTrain;
  private SpeedControllerGroup leftDrive;
  private SpeedControllerGroup rightDrive;
  private Compressor compressor;
  private DoubleSolenoid firstSolenoid, secondSolenoid;


  @Override
  public void robotInit() {
    masterLeftMotor = new PWMVictorSPX(masterLeftPort);
    slaveLeftMotor = new PWMVictorSPX(slaveLeftPort);
    masterRightMotor = new PWMVictorSPX(masterRightPort);
    slaveRightMotor = new PWMVictorSPX(slaveRightPort);
    driverController = new XboxController(controllerPort);
    leftDrive =  new SpeedControllerGroup(masterLeftMotor, slaveLeftMotor);
    rightDrive = new SpeedControllerGroup(masterRightMotor, slaveRightMotor);
    driveTrain = new DifferentialDrive(leftDrive, rightDrive);
    compressor = new Compressor(compressorPort);
    firstSolenoid = new DoubleSolenoid(0, 2);
    secondSolenoid = new DoubleSolenoid(1, 3);

    // This code here is used to control the camera or the robot vision
    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(1366, 768);
      
      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
      
      Mat source = new Mat();
      Mat output = new Mat();
      
      while(!Thread.interrupted()) {
          cvSink.grabFrame(source);
          Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
          outputStream.putFrame(output);
      }
  }).start();
}

  // Running forward or backward depended on the vertical value of the joystick
  // Running left or right depended on the horizontal value of the joystick
  // Note: Check the direction of the robot to see if the robot runs in the right direction
  // If the robot turns left, the left side will run backward and the right side will run forward
  // If the robot turns right, the left side will run forward and the right side will run backward
  // Changing the direction of the robot to left or right need to be done first
  // Running forward and backward will be done after the robot changes it horizontal direction
  // This code from here will control the pneumatic system

  private void forwardPiston(){
    firstSolenoid.set(DoubleSolenoid.Value.kForward);
    secondSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  private void reversePiston(){
    firstSolenoid.set(DoubleSolenoid.Value.kReverse);
    secondSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  private boolean checkJoystickZone(double controllerVal){
    if (controllerVal < -joystickDeathZone || controllerVal > joystickDeathZone){
      return true;
    }
    else {
      return false;
  }
}

  @Override
  public void teleopPeriodic() {
    // This code is used to control robot driving
    if (checkJoystickZone(driverController.getRawAxis(1))){
      driveTrain.tankDrive(-driverController.getRawAxis(1), -driverController.getRawAxis(1));
    }
    if (checkJoystickZone(driverController.getRawAxis(4))){
      driveTrain.tankDrive(driverController.getRawAxis(4), -driverController.getRawAxis(4));
    }

    // This code here is used to control pneumatic system
    if (driverController.getAButtonPressed()){
      forwardPiston();
    }

    if (driverController.getAButtonReleased()){
      reversePiston();
    }

    // This code here is used to control the air compressor
    if (driverController.getBButtonPressed()){
     compressor.start();
     compressor.setClosedLoopControl(true);
    }

    if (driverController.getBButtonReleased()){
      compressor.stop();
      compressor.setClosedLoopControl(false);
    }
  }
}

// We are missing the code for controlling the motor of the hatch catching
// We are missing the code for controlling the other two motors of the ball mechanism