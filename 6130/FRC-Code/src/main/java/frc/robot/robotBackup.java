package frc.robot;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class robotBackup extends TimedRobot {
  private static final int masterLeftPort = 0;
  private static final int slaveLeftPort = 1;
  private static final int masterRightPort = 2;
  private static final int slaveRightPort = 3;
  private static final int controllerPort = 1;
  private static final int robotFuncControllerPort = 0;
  private static final int compressorPort = 0;
  private static final int ballLauncherPort = 5;
  private static final int ballHandPort = 4;
  private static final int secondaryBallHandPort = 6;
  private static final int hatchHandPort = 7;
  private static final int limitSwitchHatchMeFPort = 1;
  private static final int limitSwitchHatchMeBPort = 0;
  private static final int limitSwitchBallMeFPort = 2;
  private static final int limitSwitchBallMeBPort = 3;

  // Making a death zone to control the joystick
  // private static final double joystickDeathZone = 0.3;

  private SpeedController masterLeftMotor, slaveLeftMotor, masterRightMotor, slaveRightMotor;
  private XboxController driverController;
  private XboxController robotFuncController;
  private DifferentialDrive driveTrain;
  private SpeedControllerGroup leftDrive;
  private SpeedControllerGroup rightDrive;
  private SpeedController ballHand, secondaryBallHand, ballLauncher;
  private SpeedController hatchHand;
  private Compressor compressor;
  private DoubleSolenoid firstSolenoid, secondSolenoid;
  private DigitalInput limitSwitchHatchMeFront, limitSwitchHatchMeBack;
  private DigitalInput limitSwitchBallMeFront, limitSwitchBallMeBack;
  private boolean compressorSts = true;
  private boolean compressorPreSts = false;
  private boolean bButtonSts;

  @Override
  public void robotInit() {
    ballHand = new PWMVictorSPX(ballHandPort);
    secondaryBallHand = new PWMVictorSPX(secondaryBallHandPort);
    ballLauncher = new PWMVictorSPX(ballLauncherPort);
    hatchHand = new Spark(hatchHandPort);
    masterLeftMotor = new PWMVictorSPX(masterLeftPort);
    slaveLeftMotor = new PWMVictorSPX(slaveLeftPort);
    masterRightMotor = new PWMVictorSPX(masterRightPort);
    slaveRightMotor = new PWMVictorSPX(slaveRightPort);
    driverController = new XboxController(controllerPort);
    robotFuncController = new XboxController(robotFuncControllerPort);
    leftDrive =  new SpeedControllerGroup(masterLeftMotor, slaveLeftMotor);
    rightDrive = new SpeedControllerGroup(masterRightMotor, slaveRightMotor);
    driveTrain = new DifferentialDrive(leftDrive, rightDrive);
    compressor = new Compressor(compressorPort);
    firstSolenoid = new DoubleSolenoid(0, 1);
    secondSolenoid = new DoubleSolenoid(2, 3);
    limitSwitchHatchMeFront = new DigitalInput(limitSwitchHatchMeFPort);
    limitSwitchHatchMeBack = new DigitalInput(limitSwitchHatchMeBPort);
    limitSwitchBallMeFront = new DigitalInput(limitSwitchBallMeFPort);
    limitSwitchBallMeBack = new DigitalInput(limitSwitchBallMeBPort);

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
  
  // This function here is used to control the pneumatic controller
  private void compressorSwitch(boolean compressorCMD){
    if (compressorCMD == true){
      compressor.start();
      compressor.setClosedLoopControl(true);
    } else {
      compressor.stop();
      compressor.setClosedLoopControl(false);
    }
  }

  private void compressorCtrl(){
    bButtonSts = robotFuncController.getBButton();

    if (bButtonSts == true && compressorPreSts == false){
      if (compressorSts == true){
        compressorSts = false;
      } else {
        compressorSts = true;
      }
    }
    compressorPreSts = bButtonSts;
    compressorSwitch(compressorSts);
  }

  @Override
  public void teleopPeriodic() {
    // This code here is used to control the robot driving
    driveTrain.tankDrive(-robotFuncController.getRawAxis(1), -robotFuncController.getRawAxis(5));

    // This code here is used to control the ball launcher
    if (robotFuncController.getBumperPressed(Hand.kLeft)){
      ballLauncher.set(-0.3);
    }

    if (robotFuncController.getBumperReleased(Hand.kLeft)){
      ballLauncher.set(0);
    }

    if (robotFuncController.getBumperPressed(Hand.kRight)){
      ballLauncher.set(1);
    }

    if (robotFuncController.getBumperReleased(Hand.kRight)){
      ballLauncher.set(0);
    }

    // This code here is used to control pneumatic system
    if (robotFuncController.getAButtonPressed()){
      forwardPiston();
    }

    if (robotFuncController.getAButtonReleased()){
      reversePiston();
    }

    // This code here is used to control the air compressor
    compressorCtrl();

    // This code here is used to control the hand of the ball launcher mechanism
    // We reduce the speed of the motor down to reduce the heat
    if (robotFuncController.getYButton() == true){
      if (!limitSwitchBallMeFront.get()){
        ballHand.set(0);
        secondaryBallHand.set(0);
      } else {
        ballHand.set(1);
        secondaryBallHand.set(1);
      }
    }
    
    if (robotFuncController.getXButton() == true){
      if (!limitSwitchBallMeBack.get()){
        ballHand.set(0);
        secondaryBallHand.set(0);
      } else {
        ballHand.set(-1);
        secondaryBallHand.set(-1);
      }
    }
    
    if (robotFuncController.getXButtonReleased() || robotFuncController.getYButtonReleased()){
      ballHand.set(0);
      secondaryBallHand.set(0);
    }

    // This code here is used to control the hand of the hatch mechanism
    if (robotFuncController.getPOV(0) == 0){
      if (!limitSwitchHatchMeFront.get()){
        hatchHand.set(0);
      } else{
        hatchHand.set(1);
      }
    }
  
    if (robotFuncController.getPOV(0) == 180){
      if (!limitSwitchHatchMeBack.get()){
        hatchHand.set(0);
      } else{
        hatchHand.set(-1);
      }
    }

    if (robotFuncController.getPOV(0) == -1){
      hatchHand.set(0);
    }
  }
}