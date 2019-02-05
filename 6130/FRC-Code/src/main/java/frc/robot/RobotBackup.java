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
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class RobotBackup extends TimedRobot {
  private static final int masterLeftPort = 0;
  private static final int slaveLeftPort = 1;
  private static final int masterRightPort = 2;
  private static final int slaveRightPort = 3;
  private static final int controllerPort = 0;
  private static final int compressorPort = 0;
  private static final int ballLauncherPort = 5;
  private static final int ballHandPort = 4;
  private static final int hatchHandPort = 6;

  // Making a death zone to control the joystick
  private static final double joystickDeathZone = 0.3;

  private SpeedController masterLeftMotor, slaveLeftMotor, masterRightMotor, slaveRightMotor;
  private XboxController driverController;
  private DifferentialDrive driveTrain, ballLauncher;
  private SpeedControllerGroup leftDrive;
  private SpeedControllerGroup rightDrive;
  private SpeedController ballHand, ballLauncherMotor;
  private SpeedController hatchHand;
  private Compressor compressor;
  private DoubleSolenoid firstSolenoid, secondSolenoid;

  @Override
  public void robotInit() {
    ballHand = new PWMVictorSPX(ballHandPort);
    ballLauncherMotor = new PWMVictorSPX(ballLauncherPort);
    hatchHand = new PWMVictorSPX(hatchHandPort);
    masterLeftMotor = new PWMVictorSPX(masterLeftPort);
    slaveLeftMotor = new PWMVictorSPX(slaveLeftPort);
    masterRightMotor = new PWMVictorSPX(masterRightPort);
    slaveRightMotor = new PWMVictorSPX(slaveRightPort);
    driverController = new XboxController(controllerPort);
    leftDrive =  new SpeedControllerGroup(masterLeftMotor, slaveLeftMotor);
    rightDrive = new SpeedControllerGroup(masterRightMotor, slaveRightMotor);
    driveTrain = new DifferentialDrive(leftDrive, rightDrive);
    ballLauncher = new DifferentialDrive(ballLauncherMotor, ballLauncherMotor);
    compressor = new Compressor(compressorPort);
    firstSolenoid = new DoubleSolenoid(0, 1);
    secondSolenoid = new DoubleSolenoid(2, 3);

    // This code here is used to control the camera or the robot vision
    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(1920, 1080);
      camera.setBrightness(1);
      camera.setFPS(60);
      
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

  // Check joystcick zone is used in order to operate the robot driving command
  private boolean checkJoystickZone(double controllerVal){
    if (controllerVal < -joystickDeathZone || controllerVal > joystickDeathZone){
      return true;
    }
    else {
      return false;
    }
  }

  // This function here is used to control the robot in normal mode (RC DRIVING MODE)
  private void rcDriving(){
    if (checkJoystickZone(driverController.getRawAxis(1))){
      driveTrain.tankDrive(-driverController.getRawAxis(1), -driverController.getRawAxis(1));
    }
    if (checkJoystickZone(driverController.getRawAxis(4))){
      driveTrain.tankDrive(driverController.getRawAxis(4), -driverController.getRawAxis(4));
    }
  }

  // Check POV status will return true to control the hand of the hatch mechanism if it return true
  private boolean checkPOVsts(XboxController controller, int port){
    if (driverController.getPOV(port) != -1){
      return true;
    }
    return false;
  }

  @Override
  public void teleopPeriodic() {
    // This code here is used to control the robot driving
    driveTrain.tankDrive(-driverController.getRawAxis(1), -driverController.getRawAxis(5));

    // This code here is used to control the robot driving in RC mode
    if (driverController.getBumperPressed(Hand.kRight)){
        rcDriving();
    }
   
    // This code here is used to shoot the ball from the ball launcher
    ballLauncher.tankDrive(-driverController.getTriggerAxis(Hand.kRight), -driverController.getTriggerAxis(Hand.kLeft));

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

    // This code here is used to control the hand of the ball launcher mechanism
    // We reduce the speed of the motor down to reduce the heat
    if (driverController.getYButtonPressed()){
      ballHand.set(0.5);
    }

    if (driverController.getYButtonReleased()){
      ballHand.set(0);
    }

    if (driverController.getXButtonPressed()){
      ballHand.set(-0.5);
    }

    if (driverController.getXButtonReleased()){
      ballHand.set(0);
    }

    // This code here is used to control the hand of the hatch mechanism
    if (checkPOVsts(driverController, 0)){
      hatchHand.set(0.5);
    }

    if (checkPOVsts(driverController, 0) == false){
      hatchHand.set(0);
    }

    if (checkPOVsts(driverController, 2)){
      hatchHand.set(-0.5);
    }

    if (checkPOVsts(driverController, 2) == false){
      hatchHand.set(0);
    }
  }
}

// Test this code with the robot for tomorrow
// If the code work, include the function to control the robot in like the original driving