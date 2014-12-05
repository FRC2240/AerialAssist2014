/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Encoder;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot {
    
    // Constants
    private final double RANGEFINDER_SCALE_FACTOR = (1024/5.0);
    private final double SHOOTING_DISTANCE = (194 - 24); //
    private final double AUTONOMOUS_SPEED = 1.0;
    private final double SHOOTING_FAILSAFE = 6.0;
    private final int    AUTONOMOUS_BRIGHTNESS = 3;
    private final int    TELEOP_BRIGHTNESS = 75;
    private final double GATHER_SPEED = 1.0;
    private final int    HALO_DRIVE = 1;
    private final int    TANK_DRIVE = 2;
    private final double DEADBAND_AMOUNT = 0.05;
    private final double ENCODER_MAX_PERIOD = 0.1;
    private final double ENCODER_MIN_RATE = 10.0;
    private final double ENCODER_DISTANCE_PER_PULSE = 0.05; //(39.0/817.0); // inches/count
    private final int    ENCODER_SAMPLES_TO_AVERAGE = 7;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        driveTrain  = new RobotDrive(1,2);
        Stick       = new Joystick(1);
        rangeFinder = new AnalogChannel(2);
        gyro        = new Gyro(1);
        timer       = new Timer();
        DriveType   = TANK_DRIVE;

        compressor  = new Compressor(1,1);
        LowGear     = new Solenoid(1);
        HighGear    = new Solenoid(2);
        GatherUp    = new Solenoid(3);
        GatherDown  = new Solenoid(4);
        Gather      = new Talon(3);
        Shoot1      = new Talon(4);
        Shoot2      = new Talon(5);
        LeftEncoder = new Encoder(2,3, true, Encoder.EncodingType.k4X);
        RightEncoder= new Encoder(4,5, false, Encoder.EncodingType.k4X); 
        detector    = new TargetDetector();
        detector.init();       
     
        // Initialize encoders
        LeftEncoder.setMaxPeriod(ENCODER_MAX_PERIOD);
        LeftEncoder.setMinRate(ENCODER_MIN_RATE);
        LeftEncoder.setDistancePerPulse(ENCODER_DISTANCE_PER_PULSE);
        LeftEncoder.setSamplesToAverage(ENCODER_SAMPLES_TO_AVERAGE);
        
        RightEncoder.setMaxPeriod(ENCODER_MAX_PERIOD);
        RightEncoder.setMinRate(ENCODER_MIN_RATE);
        RightEncoder.setDistancePerPulse(ENCODER_DISTANCE_PER_PULSE);
        RightEncoder.setSamplesToAverage(ENCODER_SAMPLES_TO_AVERAGE);   
        
        LeftEncoder.start();
        RightEncoder.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        boolean isDrivingDone = false;
        
        // Drive until we reach the low goal
        double d = getDistance ();
        if (d < SHOOTING_DISTANCE) {
          driveTrain.drive (-AUTONOMOUS_SPEED, 0);
        }
        else {
          driveTrain.drive (0, 0);
          isDrivingDone = true;
          System.out.println("Autonomous driving complete");
        }
        
        if (AutonmousComplete) {
            // We're done, nothing else to do for now...
            return;
        }
        
        // Look for target
        if (!isTargetHot) {
            isTargetHot = detector.isTargetHot();
        }
        
        if (timer.get() >= SHOOTING_FAILSAFE) {
            isFailsafe = true;
        }
       
        // Shhot the ball when:
        // 1) We're finished driving, and...
        // 2) The target is hot or the timer expired
        if (isDrivingDone) {
           if (isTargetHot || isFailsafe) {
               // Shoot!
               shootHighGoal();
               Timer.delay(0.75);
               stopShooter();
               
               AutonmousComplete = true;
               System.out.println("Autonomous shoot");
           }
        }
    } 
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        SmartDashboard.putNumber("Distance", getDistance());

        // Select Tank-style Drive
        if (Stick.getRawButton(8)) {
            DriveType = TANK_DRIVE;
            System.out.println("Switching to Tank Drive");
        }
        
        // Selct Halo-style Drive
        if (Stick.getRawButton(7)) {
            DriveType = HALO_DRIVE;
            System.out.println("Switching to Halo Drive");            
        }
        
        // Do the drive!
        if (DriveType == HALO_DRIVE) {
            HaloDrive();
        } else {
            TankDrive();
        }
        
        // Axis used for gather/pass
        double gather = Stick.getRawAxis(3);
        
        // Gather the ball
        if (gather > 0.1) {
            gatherMotor(GATHER_SPEED);
            gatherDown();
        } else if (gather < -0.1) {
            gatherMotor(-GATHER_SPEED);
            gatherUp();
        } else {
            gatherMotor(0.0);
        }
        
        // Gather the ball (another way to do it)
        if (Stick.getRawButton(3)) {
            gatherMotor(GATHER_SPEED);
        }
           
        if (Stick.getRawButton(2)) {
            gatherMotor(-GATHER_SPEED);
        }
        
        // Set high gear
        if (Stick.getRawButton(6)) {
            if (!HighGear.get()) {
                highGear();
            }
        }
        
        // Set low gear
        if (Stick.getRawButton(5)) {
            if (!LowGear.get()) {
                lowGear();
            }
        }
        
        // Shoot the ball
        if (Stick.getRawButton(4)) {
            Shoot1.set(-1.0);
            Shoot2.set(1.0);
        } else {
            Shoot2.set(0);
            Shoot1.set(0);  
        }
    }
     
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        
        driveTrain.drive (AUTONOMOUS_SPEED, 0);
  
        if (timer.get() >= 5.0) {
            driveTrain.drive (0.0, 0);
            return;
        }
        
        System.out.println("Right Distance = " + RightEncoder.getDistance());
        System.out.println("Right Count    = " + RightEncoder.get());
        System.out.println("Right RawCount = " + RightEncoder.getRaw());
        
        System.out.println("Left Distance = " + LeftEncoder.getDistance());
        System.out.println("Left Count    = " + LeftEncoder.get());
        System.out.println("Left RawCount = " + LeftEncoder.getRaw());       
     
       
        /*Timer.delay(1.0);
        System.out.println("gyro  = " + gyro.getAngle());
        System.out.println("range = " + getDistance());
        
        boolean[] buttons;
        double[] axes;
        buttons = new boolean[11];
        axes = new double[7];
        
        int i;
        for (i=1; i<=10; i++) {
            buttons[i] = Stick.getRawButton(i);
            System.out.println("Button " + i + " : = " + buttons[i]);
        }
        for (i=1; i<=6; i++) {
            axes[i] = Stick.getRawAxis(i);
            System.out.println("Axis " + i + " : = " + axes[i]);
        }
        System.out.println("\n");
        */
    }
    
    public void disabledInit () {
        System.out.println("disabledInit()");
        compressor.stop();
    }
    
    public void autonomousInit () {
        System.out.println("autonomousInit()");
        timer.start ();
        AutonmousComplete = false;
        AxisCamera.getInstance().writeBrightness(AUTONOMOUS_BRIGHTNESS);
        gatherUp();
        highGear();
        LeftEncoder.reset();
        RightEncoder.reset();
        
        isTargetHot = false;
        isFailsafe  = false;
    }
    
    public void teleopInit () {
        System.out.println("teleopInit()");
        AxisCamera.getInstance().writeBrightness(TELEOP_BRIGHTNESS);
        compressor.start();
        gatherDown();
        RightEncoder.reset();
        LeftEncoder.reset();
    }
    
    public void testInit () {
        System.out.println("testInit()");
        
        LeftEncoder.reset();
        RightEncoder.reset();
        
        timer.reset();
        timer.start ();
        highGear();
    }   
    
    private void lowGear() {
        System.out.println("Switching to low gear");
        HighGear.set(false);
        LowGear.set(true);
    }
    
    private void highGear () {
        System.out.println("Switching to high gear");
        HighGear.set(true);
        LowGear.set(false);
    }
    
    private void gatherUp() {
       if (!GatherUp.get()) {
            GatherUp.set(true);
            GatherDown.set(false);
       }
       //Gather.set(speed);
       System.out.println("gatherUp");
    }
    
    private void gatherDown() {
        if (GatherUp.get()) {
            GatherUp.set(false);
            GatherDown.set(true);
        }
        //Gather.set(speed);
        System.out.println("gatherDown");
    }
    
    private void gatherMotor(double speed){
        Gather.set(speed);
    }
    private void shootTruss(){
        //TO DO
    }
    
    // Intended for autonomous only
    private void shootHighGoal(){
          Shoot1.set(-1.0);
          Shoot2.set(1.0);
    }
    
    private void stopShooter() {
          Shoot1.set(0.0);
          Shoot2.set(0.0);        
    }
    
    // Intended for autonomous only
    private void shootLowGoal (){
        // Eject the ball
        gatherMotor(-1.0);
    }
    
    // Halo Drive
    // Get joystick values and translate them to motor values to pertform
    // Halo-style drive
    private void HaloDrive() {
        // Get axis values
        double turn     = deadBand(Stick.getRawAxis(4));
        double throttle = deadBand(-Stick.getRawAxis(2));
  
        // Translate to motor values
        double leftMotor  = (throttle + turn);
        double rightMotor = (throttle - turn);
        
        // Scale results (keep motor values between -1.0 and +1.0
        double scale = 1.0;
        if (Math.abs(leftMotor) > scale) {
            scale = Math.abs(leftMotor);
        }
        if (Math.abs(rightMotor) > scale) {
            scale = Math.abs(rightMotor);
        }
        
        leftMotor  = leftMotor/scale;
        rightMotor = rightMotor/scale;
        
        // Set motor speeds
        driveTrain.setLeftRightMotorOutputs(rightMotor, leftMotor);     
    }
    
    // Apply deadband filter to axis value
    private double deadBand(double AxisValue ){
        if (Math.abs(AxisValue) < DEADBAND_AMOUNT) {
           return 0;
        } else {
           return AxisValue;
        }
    }
    
    // Tank Drive
    // Get joystick values and translate them to motor values to perform
    // tank-style drive
    private void TankDrive() {
        double left = deadBand(-Stick.getRawAxis(5));
        double right  = deadBand(-Stick.getRawAxis(2));
                
        driveTrain.setLeftRightMotorOutputs(left, right);
    }
    
    private double getRange() {
        double v = rangeFinder.getAverageVoltage();
        return (v*RANGEFINDER_SCALE_FACTOR);
    }
    
    private double getDistance() {
        double left = LeftEncoder.getDistance();
        double right = RightEncoder.getDistance();
        return (left + right)/2.0;
    }
     
    private RobotDrive      driveTrain;
    private Joystick        Stick;
    private AnalogChannel   rangeFinder;
    private Gyro            gyro;
    private Timer           timer;
    private boolean         AutonmousComplete;
    private TargetDetector  detector; 
    private Compressor      compressor;
    private Solenoid        HighGear; 
    private Solenoid        LowGear;
    private Solenoid        GatherUp;
    private Solenoid        GatherDown;
    private Talon           Gather;
    private int             DriveType;
    private Talon           Shoot1;
    private Talon           Shoot2;
    private Encoder         LeftEncoder;
    private Encoder         RightEncoder;
    private boolean         isTargetHot;
    private boolean         isFailsafe;
    
    //private double throttleLimiter = leftStick.getThrottle();
  }            