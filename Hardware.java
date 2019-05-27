// ----------------------------------------------------------
// Copyright (c) 2017-2018 GaCo FRC Team 1629
//
// Date       Version Comment
// 2019-02-09 1.0  Original verion as a Sample class
// 2019-03-15 1.1  Week 3 upgrades: Jack functions.  New button config
// 2019-03-16 1.2  Updated auto speed scale and 2 speeds in Hab to cargo front
// 2019-03-16 1.3  fixed the jack offset for it to be zero.
// 2019-03-16 1.4  push buttons slower and little wheel full power
// 2019-03-16 1.5  Increased timeout and push power on habto cargo front
// 2019-03-16 1.6  Lift drop enhanced for cargo collection.  Slower Hab push.
// 2019-03-17-1.7  Loader heights decreased by 2 inches. 
// 2019-03-17-1.8  increaed the power threshold from 7.5 amps to 10 amps
// 2019-03-17-1.9  Added peak current pause
// 2019-03-21-1.10 Duplicated height buttons Adding Vectordrive
// 2019-04-03-2.0  New for District Comp
//                 Don't set auto from left hand controls
//                 Use new target Enums when determining approach headinghabtofront
// 2019-04-08-2.2  Fater driving and reduced timeout on 
// 2019-04-08-2.3  Added short push after collecting hatch Telop only
// 2019-04-11-2.4 Increased drive till contact amps and decreased climber speed
// 2019-04-11-2.5 Reduced timeout and distance on front to hab
// 2019-04-11.2.6 Increased auto loader push & dec current lim to 8.25
// 2019-04-11.2.7 Increased auto loader raise heigt & time and push 
// 2019-04-11.2.8 Added left/right shift on loader
// 2019-04-11.2.9 Stronger shift and new feed buttons (9,10 and tilt down)
// 2019-04-11.2.A Shorter timeouts on loader
// 2019-04-18.3.0 Uses updated motor controller API
//                Updated Limelight code with larger target recognition
// 2019-04-24.1 Updated the limit on current for the clibing motor
// 2-19-04-25 3.2  Increased time and distance on hab to cargo front.
// 2019-04-25 3.3 Changed the code so it only uses limelight when tracking in teliop
//2019-04-25 3.4 increased the ampage limity on drive till contact
// 2019-04-26 3.6 decreased distance on loader to cargo ship
// 2019-04-26 3.7 restored to same values as 3.4
// ----------------------------------------------------------
//
// ----------------------------------------------------------
package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Hardware {

  //  #####   PUBLIC DATA  #####
  public final double  REVISION    =  3.7;
  
  // ===================  Hardware Interfaces	 =============
  private Robot myRobot = null;

  public DigitalInput liftLimitTop;
  public DigitalInput liftLimitBottom;
  public DigitalInput jackLimitUp;

  public AnalogInput  liftStringPot;
  public Joystick 	  pilotGamepad;
  public Joystick 	  copilotLH;
  public Joystick 	  copilotRH;

  public PowerDistributionPanel pdp;
  public DoubleSolenoid intakeExtend;
  public DoubleSolenoid intakeLower;
  public DoubleSolenoid intakeEject;

  public MecanumDrive robot;

  public TalonSRX liftMotor1;
  public TalonSRX liftMotor2;
  public TalonSRX intakeMotor;
  public TalonSRX jackWheel;
 
  private CANSparkMax leftFrontDrive ;
  private CANSparkMax rightFrontDrive ;
  private CANSparkMax leftBackDrive ;
  private CANSparkMax rightBackDrive ;

  public CANSparkMax jackLifter ;

  private CANEncoder leftFrontEncoder ;
  private CANEncoder rightFrontEncoder ;
  private CANEncoder leftBackEncoder ;
  private CANEncoder rightBackEncoder ;
  public CANEncoder jackLifterEncoder ;

  //  Operator buttons (Some have dual purposes)
  public boolean PB_PushForward = false;
  public boolean PB_PushBackwards = false;
  public boolean PB_ApproachZero = false;

  public boolean PB_LiftRocketHatchLow = false;
  public boolean PB_LiftRocketHatchMid = false;
  public boolean PB_LiftRocketHatchHigh = false;
  public boolean PB_LiftRocketCargoLow = false;
  public boolean PB_LiftRocketCargoMid = false;
  public boolean PB_LiftRocketCargoHigh = false;
  public boolean PB_LiftLoaderShipCargo = false;
  public boolean PB_LiftUseSetpoint = false;
  public boolean PB_LiftManualUp = false;
  public boolean PB_LiftManualDown = false;

  public boolean PB_CollectorExtend = false;
  public boolean PB_CollectorRetract = false;
  public boolean PB_CollectorEject = false;
  public boolean PB_CollectorTiltDown = false;
  public boolean PB_CollectorTiltUp = false;
  public boolean PB_CollectorFeedIn = false;
  public boolean PB_CollectorFeedOut = false;
  public boolean PB_ResetGyro = false;

  public boolean PB_AutoScore = false;
  public boolean PB_CollectHatch = false;

  public boolean PB_AssistRocketHatchLow = false;
  public boolean PB_AssistRocketHatchMid = false;
  public boolean PB_AssistRocketHatchHigh = false;
  public boolean PB_AssistShipHatch = false;
  public boolean PB_AssistRocketCargoLow = false;
  public boolean PB_AssistRocketCargoMid = false;
  public boolean PB_AssistRocketCargoHigh = false;
  public boolean PB_DefenseEnable = false;
  
  public boolean PB_EndGameEnable = false;
  public boolean PB_EndGameExtendJack = false;
  public boolean PB_EndGameRetractJack = false;
  public boolean PB_EndGameRetractJackOverride = false;
  public boolean PB_EndGameClimbLow = false;
  public boolean PB_EndGameClimbHigh = false;
 
    
  //  General Variables
  public boolean targetLocked; //
  public double currentHeading;  // Current Gyro heading
  public double currentPitch;  // Current Gyro pitch.  +ve = front up
  public double headingLock;     // Current locked heading (from Gyro)
  public double axialInches = 0; // used to track motion
  public double lateralInches = 0;
  public double vectorInches = 0;
  public double maxDriveCurrent = 0;
  public double yawDegrees = 0;
  public double liftHeight;
  public double liftSetPoint;
  public boolean liftInPosition;
  public double jackOffset;
  public double jackHeight;
  public double pitchOffset;
  public boolean autoAlarm;

  //  #####   CONSTANTS  #####
  public static ArrayList<Targets> RocketHatches = new ArrayList <> (Arrays.asList(
    Targets.RR1, Targets.RR3, 
    Targets.LR1, Targets.LR3 ));
  
  public static ArrayList<Targets> RocketCargo = new ArrayList <> (Arrays.asList(
    Targets.RR2, 
    Targets.LR2 ));
  
  public static ArrayList<Targets> ShipTargets = new ArrayList <> (Arrays.asList(
    Targets.RC1, Targets.RC2, Targets.RC3, Targets.RC4, 
    Targets.LC1, Targets.LC2, Targets.LC3, Targets.LC4 ));
  
  public final int      SMART_CURRENT_LIMIT = 45;
  public final double   SECONDARY_CURRENT_LIMIT = 55;
  public final int      kPeakCurrentDurationMills = 0;

  public final double  AXIAL_SCALE          =  0.7;
  public final double  LATERAL_SCALE        =  0.7;
  public final double  YAW_SCALE            =  0.5;
  
  public final double  SAFE_POWER           =  1.0; //was .9
  public final double  GYRO_SCALE           =  1.01;
  
  public final double  HEIGHT_LOADER_HATCH  =  7.5;
  public final double  HEIGHT_LOADERSHIP_CARGO  = 33.0;
  public final double  HEIGHT_SHIP_HATCH    =  7.5;
  public final double  HEIGHT_ROCKET_HATCH1 =  7.5;
  public final double  HEIGHT_ROCKET_HATCH2 = 36.5;
  public final double  HEIGHT_ROCKET_HATCH3 = 64.5;
  public final double  HEIGHT_ROCKET_CARGO1 = 17.0;
  public final double  HEIGHT_ROCKET_CARGO2 = 44.0;
  public final double  HEIGHT_ROCKET_CARGO3 = 70.0; 
  public final double  HEIGHT_HIGH_ENDGAME  = 22.5;
  public final double  HEIGHT_LOW_ENDGAME   =  8.5;

  public final double HOLDING_POWER = 0.16;  //Least amount of power required to hold arm in place
  public final double LIFTING_POWER = 0.35;  //Least amount of power required to raise lift
  
  public final double LIFT_GAIN = 0.05;
  public final double LIFT_MAX_HEIGHT = 70;
  public final double LIFT_MIN_HEIGHT = 1;
  public final double LIFT_INCHES_PER_SECOND = 24 ;
  public final double MOTOR_CONTACT_CURRENT = 8.5 ; //was 8.25

  public final double JACK_PITCH = 1.0 / 20.0;

  private final double DESIRED_DISTANCE = 10 ; //
  private final double P1x =  0.0989;
  private final double P1y =  0.0;
  private final double P2x =  4.239;
  private final double P2y = 69.5;
  private final double LIFT_SCALE  = (P2y-P1y) / (P2x-P1x); // Rise / Run
  private final double LIFT_OFFSET = P1y - (LIFT_SCALE * P1x);

  private final double HEADING_GAIN = 0.012;  // was 0.01
  private final double MIN_HEADING_ERROR = 3;
  private final double MAX_YAW_POWER = 0.5;  // was .5
  private final double VRAMP = 0.03;  //  full speed in 33 inches was .024
  private final double INCHES_PER_AXIAL_REV = 2.08;
  private final double PUSH_POWER = 0.15; // used to push forward onto HAB

  // Limelight tracking information
  NetworkTable table;
  NetworkTableEntry tx; //Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  NetworkTableEntry tx0; //Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  NetworkTableEntry tx1; //Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  NetworkTableEntry ty; //Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
  NetworkTableEntry ta; //Target Area (0% of image to 100% of image)
  NetworkTableEntry ta0; //Target Area (0% of image to 100% of image) (large)
  NetworkTableEntry ta1; //Target Area (0% of image to 100% of image) (small)
  NetworkTableEntry tv; //Whether the limelight has any valid targets (0 or 1)
  NetworkTableEntry thor; //Horizontal sidelength of the rough bounding box (0 - 320 pixels)
  NetworkTableEntry tvert; //Vertical sidelength of the rough bounding box (0 - 320 pixels)
  NetworkTableEntry tvert0;
  NetworkTableEntry tvert1;
  NetworkTableEntry led; //
  NetworkTableEntry snapshot; //
  NetworkTableEntry stream;

  // Vision processing varibales
  private double targetOffAxis;   //how far off center using tx
  private double targetOffAxisDistance ;
  private double targetOffAngle;  //horizontal off perpendicular using tx0 and tx1
  private double targetRange;     //how far away is target

  // actuator variables
  private boolean autoLevel;
  private double liftPower;
  public  boolean endgameActive;
  public  boolean oldAuto;
 
  //  Driving variables
  private double driveAxial;      // Power for fwd/rev driving
  private double driveLateral;    // Power for left-right strafing
  private double driveYaw;        // Power for rotating
  private boolean turning;        // We are still turning
  private boolean prototypeRobot = false;

  public TargetType assistTargetType;
  public double  assistLiftTransitHeight;
  public double  assistLiftScoreHeight;
  public double  flipFactor;     // used to flip heading and strafe values
  

  double LF1  = 0;
  double RF1  = 0;
  double LB1  = 0;
  double RB1  = 0;

  private Timer timer; // timer
  private AHRS ahrs;   //gyro

  // ###############################################################################
  // ###############################################################################

  public void init(Robot aRobot) {

    myRobot = aRobot;
    leftFrontDrive = new CANSparkMax(11, MotorType.kBrushless);
    leftBackDrive = new CANSparkMax(12, MotorType.kBrushless);
    rightBackDrive  = new CANSparkMax(13, MotorType.kBrushless);
    rightFrontDrive  = new CANSparkMax(14, MotorType.kBrushless);

    leftFrontDrive.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
    leftFrontDrive.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT, kPeakCurrentDurationMills);

    leftBackDrive.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
    leftBackDrive.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT, kPeakCurrentDurationMills);

    rightFrontDrive.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
    rightFrontDrive.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT, kPeakCurrentDurationMills);
    
    rightBackDrive.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
    rightBackDrive.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT, kPeakCurrentDurationMills);
    

    jackLifter  = new CANSparkMax(20, MotorType.kBrushless);

   jackLifter.setSmartCurrentLimit(80);
    jackLifter.setSecondaryCurrentLimit(80, kPeakCurrentDurationMills);

    jackLifter.set(0);
    jackLifter.setInverted(true);


    liftMotor1 = new TalonSRX(15);
    liftMotor1.set(ControlMode.PercentOutput, 0);
    liftMotor1.setInverted(true);

    liftMotor2 = new TalonSRX(16);
    liftMotor2.set(ControlMode.PercentOutput, 0);
    liftMotor2.setInverted(true);

    intakeMotor = new TalonSRX(17);
    intakeMotor.set(ControlMode.PercentOutput, 0);
    intakeMotor.setInverted(true);

    jackWheel = new TalonSRX(18);
    jackWheel.set(ControlMode.PercentOutput, 0);
    jackWheel.setInverted(false);

    pdp = new PowerDistributionPanel();

    liftLimitTop    = new DigitalInput(0);
    liftLimitBottom = new DigitalInput(1);
    jackLimitUp     = new DigitalInput(2);
    liftStringPot   = new AnalogInput(0);

    intakeExtend = new DoubleSolenoid(1, 5, 4);
    intakeExtend.set(DoubleSolenoid.Value.kReverse);
  
    intakeLower  = new DoubleSolenoid(1, 3, 2);
    intakeLower.set(DoubleSolenoid.Value.kReverse);

    intakeEject  = new DoubleSolenoid(1, 1, 0);
    intakeEject.set(DoubleSolenoid.Value.kReverse);

    timer = new Timer();
    timer.start();

    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    robot = new MecanumDrive(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);
    leftFrontEncoder = new CANEncoder(leftFrontDrive);
    rightFrontEncoder = new CANEncoder(rightFrontDrive);
    leftBackEncoder = new CANEncoder(leftBackDrive);
    rightBackEncoder = new CANEncoder(rightBackDrive);

    jackLifterEncoder = new CANEncoder(jackLifter);
    jackOffset = jackLifterEncoder.getPosition();

    pilotGamepad = new Joystick(0);
    copilotLH = new Joystick(1);
    copilotRH = new Joystick(2);

    // Network variables
    table = NetworkTableInstance.getDefault().getTable("limelight");
    targetLocked = false;
    tx = table.getEntry("tx");
    tx0 = table.getEntry("tx0");
    tx1 = table.getEntry("tx1");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    ta0 = table.getEntry("ta0");
    ta1 = table.getEntry("ta1");
    tv = table.getEntry("tv");
    thor = table.getEntry("thor");
    tvert = table.getEntry("tvert");
    tvert0 = table.getEntry("tvert0");
    tvert1 =table.getEntry("tvert1");
    led = table.getEntry("ledMode");
    stream = table.getEntry("stream");
    snapshot = table.getEntry("snapshot");
    led.setValue(0);
    stream.setValue(2);
   
    targetOffAxis=0;
    targetOffAngle=0; //positive off set means largest angle is on the right
    targetRange=0;

    driveAxial      = 0;      // Power for fwd/rev driving
    driveLateral    = 0;    // Power for left-right strafing
    driveYaw        = 0;        // Power for rotating
    liftPower       = 0;
    currentHeading  = 0;  // Current Gyro heading
    headingLock     = 0;     // Current locked heading (from Gyro)
    turning         = false;     // We are not turning
    flipFactor      = 1.0;    // used to flip heading and strafe values
    jackOffset      = 0;
    endgameActive   = false;
    prototypeRobot  = false;
    oldAuto         = false;
    autoAlarm       = false;

    assistTargetType        = TargetType.ROCKET_HATCH;
    assistLiftTransitHeight = HEIGHT_ROCKET_HATCH1;
    assistLiftScoreHeight   = HEIGHT_ROCKET_HATCH1;
    
    // Reset Gyro to 0
    resetHeading();

    periodic();
    liftSetPoint   = liftHeight;
    autoLevel      = false;
    liftInPosition = false;
  }

  //  ############################################################################################
  //  periodic style methods
  //  ############################################################################################

  // Run all periodic tasks
  public  void periodic() {
    sensors();
    tracking();
    runLiftControl();
    dashboard();
  }

  public void sensors() {

    // button pad
    //     270
    // 180 -1   0
    //      90
    //
    //  A2  B5  
    //  B1  B3
    //  B2  B4
    //  A3  B6
    
    // Process CoPilot Tilt Buttons
    PB_CollectorTiltDown      = (copilotRH.getPOV() == 90);
    PB_CollectorTiltUp        = (copilotRH.getPOV() == 270);

    // Process Pilot Buttons
    PB_CollectorEject         = pilotGamepad.getRawButton(6);
    PB_CollectorFeedIn        = (pilotGamepad.getRawButton(2) || copilotRH.getRawButton(9) || PB_CollectorTiltDown);
    PB_CollectorFeedOut       = (pilotGamepad.getRawButton(4) || copilotRH.getRawButton(10));
    PB_PushForward            = (pilotGamepad.getPOV() == 0);
    PB_PushBackwards          = (pilotGamepad.getPOV() == 180);
    PB_ApproachZero           = pilotGamepad.getRawButton(3);

    PB_CollectHatch           = pilotGamepad.getRawButton(7);
    PB_AutoScore              = pilotGamepad.getRawButton(8);
    
    PB_ResetGyro              = (pilotGamepad.getRawButton(9) && pilotGamepad.getRawButton(10));

    // Process CoPilot lefthand Buttons
    PB_LiftRocketHatchLow     = (copilotLH.getRawAxis(3) > 0.5);
    PB_LiftRocketHatchMid     = copilotLH.getRawButton(2);
    PB_LiftRocketHatchHigh    = copilotLH.getRawButton(1);
    PB_LiftRocketCargoLow     = copilotLH.getRawButton(6);
    PB_LiftRocketCargoMid     = copilotLH.getRawButton(4);
    PB_LiftRocketCargoHigh    = copilotLH.getRawButton(3);
    PB_LiftLoaderShipCargo    = copilotLH.getRawButton(5);
    
    PB_LiftUseSetpoint        = (copilotLH.getPOV() == -1);
    PB_LiftManualUp           = (copilotLH.getPOV() == 270);
    PB_LiftManualDown         = (copilotLH.getPOV() == 90);

    PB_CollectorExtend        = copilotLH.getRawButton(10);
    PB_CollectorRetract       = copilotLH.getRawButton(9);

    PB_EndGameEnable          = (copilotLH.getRawAxis(2) > 0.5);

    // Process CoPilot Right hand Buttons
          
    if (PB_EndGameEnable) {
      // Only respond to end game controls.

      PB_AssistRocketHatchLow   = false;
      PB_AssistRocketHatchMid   = false;
      PB_AssistRocketHatchHigh  = false;
      PB_AssistShipHatch        = false;

      PB_AssistRocketCargoLow   = false;
      PB_AssistRocketCargoMid   = false;
      PB_AssistRocketCargoHigh  = false;
      PB_DefenseEnable        = false;

      PB_EndGameClimbLow        = copilotRH.getRawButton(6);
      PB_EndGameClimbHigh       = (copilotRH.getRawAxis(3) > 0.5);
      PB_EndGameExtendJack      = copilotRH.getRawButton(2);
      PB_EndGameRetractJack     = copilotRH.getRawButton(1);
      PB_EndGameRetractJackOverride = (copilotRH.getRawAxis(2) > 0.5);
    }
    else {  
      // Only respond to assist controls.

      if (PB_AssistRocketHatchLow   = (copilotRH.getRawAxis(3) > 0.5)) // Dont reset from Left Hand 
      {
        assistTargetType        = TargetType.ROCKET_HATCH;
        assistLiftTransitHeight = HEIGHT_ROCKET_HATCH1;
        assistLiftScoreHeight   = HEIGHT_ROCKET_HATCH1;
       }

      if (PB_AssistRocketHatchMid   = (copilotRH.getRawButton(2) || copilotLH.getRawButton(2)))
      {
        assistTargetType        = TargetType.ROCKET_HATCH;
        assistLiftTransitHeight = HEIGHT_ROCKET_HATCH1;
        assistLiftScoreHeight   = HEIGHT_ROCKET_HATCH2;
      }
        
      if (PB_AssistRocketHatchHigh  = (copilotRH.getRawButton(1) || copilotLH.getRawButton(1)))
      {
        assistTargetType        = TargetType.ROCKET_HATCH;
        assistLiftTransitHeight = HEIGHT_ROCKET_HATCH1;
        assistLiftScoreHeight   = HEIGHT_ROCKET_HATCH3;
      }
        
      if (PB_AssistShipHatch    = ((copilotRH.getRawAxis(2) > 0.5) || (copilotLH.getRawAxis(2) > 0.5)))
      {
        assistTargetType        = TargetType.SHIP_HATCH;
        assistLiftTransitHeight = HEIGHT_SHIP_HATCH;
        assistLiftScoreHeight   = HEIGHT_SHIP_HATCH;
      }

      if (PB_AssistRocketCargoLow   = (copilotRH.getRawButton(6) || copilotLH.getRawButton(6)))
      {
        assistTargetType        = TargetType.ROCKET_CARGO;
        assistLiftTransitHeight = HEIGHT_ROCKET_CARGO1;
        assistLiftScoreHeight   = HEIGHT_ROCKET_CARGO1;
      }
        
      if (PB_AssistRocketCargoMid   = (copilotRH.getRawButton(4) || copilotLH.getRawButton(4)))
      {
        assistTargetType        = TargetType.ROCKET_CARGO;
        assistLiftTransitHeight = HEIGHT_ROCKET_CARGO1;
        assistLiftScoreHeight   = HEIGHT_ROCKET_CARGO2;
      }
        
      if (PB_AssistRocketCargoHigh  = (copilotRH.getRawButton(3) || copilotLH.getRawButton(3)))
      {
        assistTargetType        = TargetType.ROCKET_CARGO;
        assistLiftTransitHeight = HEIGHT_ROCKET_CARGO1;
        assistLiftScoreHeight   = HEIGHT_ROCKET_CARGO3;
      }
        
      PB_DefenseEnable          = copilotRH.getRawButton(5);
        
      PB_EndGameClimbLow        = false;
      PB_EndGameClimbHigh       = false;
      PB_EndGameExtendJack      = false;
      PB_EndGameRetractJack     = false;
      PB_EndGameRetractJackOverride = false;
    }

    // Process attitude sensors
    currentHeading = getHeading();
    currentPitch = -(ahrs.getPitch() - pitchOffset);
   
    liftHeight = (liftStringPot.getAverageVoltage() * LIFT_SCALE) + LIFT_OFFSET;
    jackHeight = (jackLifterEncoder.getPosition() - jackOffset) * JACK_PITCH ;
    endgameActive = (jackHeight < -0.5);  // have we started extending the jack ?

    updateMotion();
 }

  public  void tracking() {
    // See if we have a target
    targetLocked =  (tv.getDouble(0) > 0);

    // Update tracking data if we have a target
    if (targetLocked) {
      // Read angle to target
      targetOffAngle =tx.getDouble(0);

      // Determine distance to target using target height
      double V0 = tvert0.getDouble(0);
      double V1 = tvert1.getDouble(0);
      double Hor = thor.getDouble(1);

      //double targetX=1/((V0 + V1)/2);
      //targetRange = (1664 * targetX) + 2.93;
      // This calculation adjusted for demo robot....  test on field
      targetRange = (3860 / Hor) - 5.178;  // was (3860 / Hor) - 5.178

      // Determine off axis distance using shapes
      targetOffAxis = (V0 - V1) / ((V0 + V1)/2);

      //is target 0 on the left
      if (tx0.getDouble(0) > tx1.getDouble(0)){
        targetOffAxis *= -1.0 ;
      }
    }
    else {
      targetOffAxis = 0;
      targetOffAngle = 0;
      targetRange = -1;
    }
  }
  
  public  void dashboard() {

    // SmartDashboard.putBoolean("Prototype", prototypeRobot);
    prototypeRobot = SmartDashboard.getBoolean("Prototype", false);
    oldAuto        = SmartDashboard.getBoolean("Old Auto", false);

    //prints off the off angle and the off axis
    SmartDashboard.putNumber("Target Axis ", targetOffAxis);
    SmartDashboard.putNumber("Target angle ", targetOffAngle);
    SmartDashboard.putNumber("OffAxisDist", targetOffAxisDistance);   
    SmartDashboard.putBoolean("Target Lock ", targetLocked);
    

    SmartDashboard.putNumber("Left front Speed", leftFrontEncoder.getVelocity());
    SmartDashboard.putNumber("Right front Speed", -rightFrontEncoder.getVelocity());
    SmartDashboard.putNumber("Left back Speed", leftBackEncoder.getVelocity());
    SmartDashboard.putNumber("Right back Speed", -rightBackEncoder.getVelocity());

    SmartDashboard.putNumber("Left front  Enc", leftFrontEncoder.getPosition());
    SmartDashboard.putNumber("Right front Enc", -rightFrontEncoder.getPosition());
    SmartDashboard.putNumber("Left Back  Enc", leftBackEncoder.getPosition());
    SmartDashboard.putNumber("Right back Enc", -rightBackEncoder.getPosition());

    SmartDashboard.putNumber("Jack Enc",jackLifterEncoder.getPosition());

    SmartDashboard.putNumber("Target Inches", targetRange);
    SmartDashboard.putNumber("Axial Power", driveAxial);
    SmartDashboard.putNumber("Lateral Power", driveLateral);
    SmartDashboard.putNumber("Yaw Power", driveYaw);
    SmartDashboard.putNumber("Jack Height", jackHeight);
    SmartDashboard.putNumber("Lift Height", liftHeight);
    SmartDashboard.putNumber("Lift Voltage", liftStringPot.getAverageVoltage());
    SmartDashboard.putNumber("Lift Power", liftPower);

    SmartDashboard.putNumber("Heading", currentHeading);
    SmartDashboard.putNumber("Pitch", currentPitch);
    SmartDashboard.putNumber("Flip Factor", flipFactor);

    SmartDashboard.putBoolean("Top Limit", liftLimitTop.get());
    SmartDashboard.putBoolean("Bottom Limit", liftLimitBottom.get());
    SmartDashboard.putBoolean("Jack Limit", jackLimitUp.get());
    
    SmartDashboard.putNumber("Lift Setpoint", liftSetPoint);
    
    SmartDashboard.putBoolean("Auto Alarm", autoAlarm);
    SmartDashboard.putNumber("Revision", REVISION);

       
  }
  
  //  ############################################################################################
  //  VISION TARGET DRIVING
  //  ############################################################################################

  //drives to vision target using image size for offAxis distance.  
  public boolean driveVision(double approachHeading){
    double approachError;
    boolean inPosition = ((targetRange < (DESIRED_DISTANCE + 4)) && (Math.abs(targetOffAxis) <= 1));

    // only drive if we have a valid range
    if(targetRange > 0){
      double rangeError = targetRange - DESIRED_DISTANCE;

      driveAxial = clip(rangeError / 55, 0.5) ;  // was 55
      driveYaw = clip(targetOffAngle / 100, 0.3);  // was 0.2 clip

      if (targetRange < 36) {
        driveLateral = clip(targetOffAxis * 2.0, .2); 
      }
      else {
        // determine off axis distance based on normalized heading
        approachError = normalizeHeading(currentHeading - approachHeading);
        
        targetOffAxisDistance = Math.sin(Math.toRadians((targetOffAngle + approachError))) * targetRange;
        driveLateral = clip( targetOffAxisDistance / 60, 0.35);  // was 50 with a  0.3 clip
      }
    }

    headingLock = currentHeading;
    return inPosition;
  }

  public double getTargetHeading() {
    double targetHeading;

    periodic();

    // Offsett current heading with target off angle to get actual heading
    if (targetLocked) {
      targetHeading = currentHeading + targetOffAngle;
    }
    else {
      targetHeading = currentHeading;
    }

    return targetHeading;
  }

  // Drive towards vision target... Adjust for left side mirror action.
  public boolean driveToTarget(Targets target, double axialDistance, double lateralDistance, double maxSpeed, double heading,
                               double approachDistance, double timeout) {
    double endTime = timer.get() + timeout;
    boolean exit = false;

    // Flip the heading and lateral signs if we are on left side.
    heading *= flipFactor ;
    lateralDistance *= flipFactor;

    // Drive the robot towards target
    resetMotion();
    while (myRobot.isEnabled() && !exit && (timer.get() <= endTime)) {

      periodic();  // get latest robot data

      if (targetLocked) {
        if ((targetRange > approachDistance) || (Math.abs(targetOffAxis) > 1) ) {
          driveVision(getApproachHeading(target) * flipFactor);  // compensate for left/right flip
        }
        else {
          exit = true;
        }
      } else {
        driveYaw = getYawPower(heading);
        driveAxial = getPowerProfile(axialDistance, maxSpeed, axialInches, VRAMP);
        driveLateral = getPowerProfile(lateralDistance, maxSpeed, lateralInches, VRAMP);

        if((driveAxial == 0) && (driveLateral == 0))
        exit = true;
      }

      // Determine rotation power required to hold heading
      driveRobot();
    }

    //if the target is not locked, turn to the heading of the target
    if (!targetLocked){
      turnToHeading(getApproachHeading(target), 1);
    }

    // Stop the robot
    stopRobot();

    return (timer.get()> endTime);
  }

  //
  public double getApproachHeading(Targets target){
    double approach = 0;

    switch(target){
      case LC1:
      case RC1:
        approach = 0;
        break;

      case RC2:
      case RC3:
      case RC4:
      case LR2:
        approach = -90;
        break;

      case LC2:
      case LC3:
      case LC4:
      case RR2:
        approach = 90;
        break;

      case LL1:
      case RL1:
        approach = 180;
        break;

      case LR1:
        approach = -30;
        break;

      case RR1:
        approach = 30;
        break;

      case LR3:
        approach = -150;
        break;

      case RR3:
        approach = 150;
        break;

      default:
        approach = 0;
        break;
    }
    return approach;
  }

  // Find the target with the approach angle closest to our current heading
  public double findClosestTargetApproach() {
    double bestApproach = 0;
    double bestError = 600;
    double normalHeading = normalizeHeading(currentHeading);
    double error;
    ArrayList<Targets> selectedTargets;

    // select the correct list of targets
    switch (assistTargetType) {
      case ROCKET_HATCH:
      default:
        selectedTargets = RocketHatches;
        break;

      case ROCKET_CARGO:
        selectedTargets = RocketCargo;
        break;

      case SHIP_CARGO:
      case SHIP_HATCH:
        selectedTargets = ShipTargets;
        break;
    }

    // iterate all the approach headings to find the one with the smallest error
    for (Targets target : selectedTargets) {
      error = Math.abs(getApproachHeading(target) - normalHeading);
      if (error < bestError) {
        bestError = error;
        bestApproach = getApproachHeading(target);
        SmartDashboard.putString ("FOUND", target.toString());
        SmartDashboard.putNumber("Approach", bestApproach);
      }
    }
      
    return (bestApproach);
  }

  public double normalizeHeading(double heading) {
    while (heading > 180)
      heading -= 360;
    while (heading < -180)
      heading += 360;

      return heading;
  }
  
  //  ############################################################################################
  //  STANDARD DRIVING & HEADING METHODS
  //  ############################################################################################

  public void setAutoSide(boolean startingOnRight) {
    flipFactor = startingOnRight ? 1.0 : -1.0 ;
  }

  //drive by joystick
  public void driveJoystick(){

    if (PB_PushForward) {
      driveAxial   = PUSH_POWER;
      driveLateral = 0;
      driveYaw     = 0;
    }
    else if (PB_PushBackwards) {
      driveAxial   = - PUSH_POWER;
      driveLateral = 0;
      driveYaw     = 0;
    }
    else {
      driveAxial = (pilotGamepad.getY() * pilotGamepad.getY()) * -Math.signum(pilotGamepad.getY());
      driveLateral = (pilotGamepad.getX() * pilotGamepad.getX()) * Math.signum(pilotGamepad.getX());
      driveYaw = (pilotGamepad.getZ() * pilotGamepad.getZ()) * Math.signum(pilotGamepad.getZ()) ;

      if (!PB_DefenseEnable) {
        driveAxial   *= AXIAL_SCALE;
        driveLateral *= LATERAL_SCALE;
        driveYaw     *= YAW_SCALE;
      }
    }

    // We are in manual driving mode
    // If the driver is turning, update heading lock for future, else lock to heading
    if (Math.abs(driveYaw) > 0.05) {
      turning = true;
      headingLock = currentHeading;
    }
    else {
      // Allow the robot to stop rotating and then lock in the heading.
      if (turning) {
        if (Math.abs(ahrs.getRawGyroZ()) < 75) {
          headingLock = getHeading();
          turning = false;
        }
      }
      driveYaw = getYawPower(headingLock);
    }

    // also control the jack wheel if it is down
    if (endgameActive) {
      // Slave the jackWheel to the forward drive
      if (driveAxial > 0.075)
        jackWheel.set(ControlMode.PercentOutput, 1 );
      else if (driveAxial < -0.075)
        jackWheel.set(ControlMode.PercentOutput, -0.25 );
      else
        jackWheel.set(ControlMode.PercentOutput, 0 );
    }
    else {
      jackWheel.set(ControlMode.PercentOutput, 0);
    }

  }

  // Drive to a lateral and axial goal distance, while holding heading.
  public  boolean driveProfile(double axialDistance, double lateralDistance, double maxSpeed, double heading, double timeout){
    double endTime = timer.get() + timeout;

    //limit speed for now
     maxSpeed *= SAFE_POWER;

    // Flip the heading and lateral signs if we are on left side.
    heading *= flipFactor ;
    lateralDistance *= flipFactor;

    resetMotion(); //remember where we started
    while (myRobot.isEnabled() && (timer.get() <= endTime)) {

      periodic();  // get latest robot data

      driveYaw = getYawPower(heading);
      driveAxial = getPowerProfile(axialDistance, maxSpeed, axialInches, VRAMP);
      driveLateral = getPowerProfile(lateralDistance, maxSpeed, lateralInches, VRAMP);
      driveRobot();
      if((driveAxial == 0) && (driveLateral == 0)){
        break;
      }
    }

    // Stop the robot
    stopRobot();

    return (timer.get() <= endTime);  // return true if we reached the target before timing out
  }

  // Drive a distance in a direction, while rotating robot to finalHeading.
  public  boolean driveVector(double vectorDistance, double maxSpeed, double direction, double finalHeading, double timeout){
    double endTime = timer.get() + timeout;
    double vectorPower = 0;

    //limit speed for now
    maxSpeed *= SAFE_POWER;

    // Flip the heading and lateral signs if we are on left side.
    direction *= flipFactor ;
    finalHeading *= flipFactor ;

    resetMotion(); //remember where we started
    while (myRobot.isEnabled() && (timer.get() <= endTime)) {

      periodic();  // get latest robot data

      driveYaw = clip(getYawPower(finalHeading), 0.2);  // slow turn rate
      vectorPower = getPowerProfile(vectorDistance, maxSpeed, vectorInches, VRAMP);
      driveLateral = vectorPower * Math.sin(direction * (Math.PI / 180.0));
      driveAxial = vectorPower * Math.cos(direction * (Math.PI / 180.0));

      driveRobot(-currentHeading);
      
      if(vectorPower == 0){
        break;
      }
    }

    // Stop the robot
    stopRobot();

    return (timer.get() <= endTime);  // return true if we reached the target before timing out
  }

  // Drive on heading until wheels stop turning.
  public  boolean driveTillContact(double axialDistance, double maxSpeed, double heading, double timeout){
    double endTime = timer.get() + timeout;
    double peakTime = timer.get() + .25;

    // Flip the heading if we are on left side.
    heading *= flipFactor ;

    resetMotion(); //remember where we started

    while (myRobot.isEnabled() && (timer.get() <= endTime)) {

      periodic();  // get latest robot data

      driveYaw = 0;// getYawPower(heading);
      driveAxial = getPowerProfile(axialDistance, maxSpeed, axialInches, VRAMP);
      // driveAxial = maxSpeed;
      driveLateral = 0;
      driveRobot();
      if((timer.get() > peakTime) && (maxDriveCurrent > MOTOR_CONTACT_CURRENT)){
        break;
      }

      if (Math.abs(driveAxial) < 0.05 ) {
        break;
      }

    }

    // Stop the robot
    stopRobot();
    return (timer.get() <= endTime);  // return true if we reached the target before timing out
  }


  //turns to the correct angle
  public void turnToHeading(double heading, double timeout) {
    double endTime = timer.get() + timeout;

    // Flip the heading if we are on left side.
    heading *= flipFactor ;

    while (myRobot.isEnabled() && (timer.get() <= endTime)) {

      periodic();
      driveRobot(0.0, 0.0, getYawPower(heading));

      // exit loop if we are close enough
      if (Math.abs(heading - getHeading()) <= MIN_HEADING_ERROR)
        break;
    }
    stopRobot();
  }

  //allowing time to lock on to the correct angle
  public void holdHeading(double heading, double timeout) {
    double endTime = timer.get() + timeout;

    // Flip the heading if we are on left side.
    heading *= flipFactor ;

    while (myRobot.isEnabled() && (timer.get() <= endTime)) {
      periodic();
      driveRobot(0.0, 0.0, getYawPower(heading));
      SmartDashboard.putNumber("holdHeading remaining", endTime -  timer.get());

    }
    stopRobot();
  }

  // allow time to just settle from any action
  public void holdPosition(double timeout) {
    double endTime = timer.get() + timeout;

    stopRobot();
    while (myRobot.isEnabled() && (timer.get() <= endTime)) {
      periodic();
    }
    stopRobot();
  }

  //try to get a better hold on te hatch cover
  public void boostGrab(double power, double timeout) {
    double endTime = timer.get() + timeout;

    while (myRobot.isEnabled() && (timer.get() <= endTime)) {
      periodic();

      // shift based on the driver buttons
      if(pilotGamepad.getRawButton(1))
        driveRobot(0, -0.2, 0.0);
      else if(pilotGamepad.getRawButton(3))
        driveRobot(0,  0.2, 0.0);
      else
        driveRobot(power, 0.0, 0.0);
    }
    stopRobot();
  }

  public void stopRobot() {
    driveRobot(0,0,0);
    headingLock = getHeading();
  }

  
  // applies the desired power to the wheels.
  public void driveRobot(double ax, double lat, double yaw) {
    driveAxial = ax;
    driveLateral = lat;
    driveYaw = yaw;
    driveRobot();
  }
   
  public void driveRobot() {
    driveRobot(0);
  }

  public void driveRobot(double gyroHeading) {
    robot.driveCartesian(driveLateral, driveAxial, driveYaw, gyroHeading);
  }

  //calculate yaw power based on disired heading
  private double getYawPower(double desired) {
    double output = (normalizeHeading(desired - getHeading())) * HEADING_GAIN;
    return (clip(output, MAX_YAW_POWER));
  }

  //resets gyro and sets headings to zero
  public void resetHeading() {
    ahrs.zeroYaw();
    headingLock = 0;
    currentHeading = 0;
    resetMotion();
  }

  //return the corrected angle of the gyro
  public double getHeading(){
    currentHeading = ahrs.getAngle() * GYRO_SCALE ;
    return  (currentHeading);
  }  //return the corrected angle of the gyro

  public double lockHeading(){
    headingLock = currentHeading;
    return  (headingLock);
  }

  // resest the measurment for any motion profile
  private void resetMotion() {
    LF1 = leftFrontEncoder.getPosition();
    RF1 = -rightFrontEncoder.getPosition();
    LB1 = leftBackEncoder.getPosition();
    RB1 = -rightBackEncoder.getPosition();
    axialInches = 0;
    lateralInches = 0;
    yawDegrees = 0;
  }

  private void updateMotion() {
    double LF2 = leftFrontEncoder.getPosition();
    double RF2 = -rightFrontEncoder.getPosition();
    double LB2 = leftBackEncoder.getPosition();
    double RB2 = -rightBackEncoder.getPosition();
    double IBL, IBR, IFL, IFR;

    double axialRevs   = ((LF2-LF1) + (RF2-RF1) + (LB2-LB1) + (RB2-RB1)) / 4;
    double lateralRevs = ((LF2-LF1) - (RF2-RF1) - (LB2-LB1) + (RB2-RB1)) / 4;

    axialInches = (axialRevs) * INCHES_PER_AXIAL_REV;
    lateralInches = (lateralRevs) * INCHES_PER_AXIAL_REV;
    vectorInches = Math.sqrt((axialInches*axialInches) + (lateralInches*lateralInches)) ;

    // read the drive motor currents based on which robot we are running
    if (prototypeRobot) {
      IFL = pdp.getCurrent(1);
      IFR = pdp.getCurrent(2);
      IBL = pdp.getCurrent(0);
      IBR = pdp.getCurrent(3);
    }
    else {
      IFL = pdp.getCurrent(1);
      IFR = pdp.getCurrent(2);
      IBL = pdp.getCurrent(14);
      IBR = pdp.getCurrent(15);
    }
       
    maxDriveCurrent = Math.max(IFL, IFR);
    maxDriveCurrent = Math.max(maxDriveCurrent, IBL);
    maxDriveCurrent = Math.max(maxDriveCurrent, IBR);

    SmartDashboard.putNumber("FL", IFL);
    SmartDashboard.putNumber("FR", IFR);
    SmartDashboard.putNumber("BL", IBL);
    SmartDashboard.putNumber("BR", IBR);
    
    SmartDashboard.putNumber("Axial Inches", axialInches);
    SmartDashboard.putNumber("Lateral Inches", lateralInches);
    SmartDashboard.putNumber("Vector Inches", vectorInches);
    SmartDashboard.putNumber("Max current", maxDriveCurrent);
  }

  // adjust power to provide smooth acc/decell curves
  private double getPowerProfile(double dTotal, double pTop, double dMotion, double vRamp) {
    double sign = Math.signum(dTotal);
    double dAcc = pTop / VRAMP;
    double udTotal = Math.abs(dTotal);
    double dConst = udTotal - dAcc;
    double udMotion = Math.abs(dMotion);
    double reqPower = 0;

    // Check to see if top speed can even be reached in time.
    if (dAcc > (udTotal / 2)) {
      dAcc = udTotal / 2;
      dConst = dAcc;
    }

    if (udMotion < dAcc)
      reqPower = 0.1 + (udMotion * vRamp);  // Add a little bit to get started
    else if (udMotion < dConst )
      reqPower = pTop;
    else if (udMotion < udTotal)
      reqPower = 0.1 + ((udTotal - udMotion) * vRamp);
    else
      reqPower = 0;

    reqPower = clip(reqPower, pTop);

    return (reqPower * sign);
  }

  //  ############################################################################################
  //  LIFT CONTROL
  //  ############################################################################################

  // Run a lift control cycle and look for IN position condition
	void runLiftControl() {
		double error = liftSetPoint - liftHeight;
		double power;

    if (autoLevel) {
      // Determine direction of required force
      if (error > 0) {
        power = (error * LIFT_GAIN) + LIFTING_POWER;
      }
      else {
        power = (error * LIFT_GAIN) + HOLDING_POWER; // reduce holding power to drop
      }

      // Make sure the limit switches prevent excess motion
      if ((power > 0) && liftLimitTop.get()) {
        power = HOLDING_POWER;
      } else if ((power < 0) && liftLimitBottom.get()) {
        power = 0;
      } else if (Math.abs(error) < 1) {
        // Apply enough force to hold lift in place
        power = HOLDING_POWER;
      }

      // Determine if lift is "inPosition" with a wider range
      liftInPosition = (Math.abs(error) <= 2.0) ;

      // Make sure power to lift is off if idle on bottom
      if (liftInPosition && liftLimitBottom.get()) {
        power = 0;
      }

      // output required lift power
      setLiftPower(clip(power, 1));
    }
	}

  void setLiftPower(double power) {
    liftPower = power;
    liftMotor1.set(ControlMode.PercentOutput, liftPower);
    liftMotor2.set(ControlMode.PercentOutput, liftPower);
  }

  void lockLift() {
    periodic();
    liftSetPoint = liftHeight; 
  }

  void enableAutoLift() {
    if (!autoLevel) {
      lockLift();
      autoLevel = true;
   }
  }

  void disableAutoLift() {
    if (autoLevel) {
      autoLevel = false;
     liftPower = 0;
     liftMotor1.set(ControlMode.PercentOutput, liftPower);
    }
  }

	// Set the new lift target position
	void setLiftHeight(double setPoint) {

		// Look to see if we need to stage the descent to the floor
		if (setPoint < LIFT_MIN_HEIGHT) {
      liftSetPoint = LIFT_MIN_HEIGHT;
    } else if (setPoint > LIFT_MAX_HEIGHT) {
      liftSetPoint = LIFT_MAX_HEIGHT;
    } else {
      liftSetPoint = setPoint;
    }
  }
  
  //  ############################################################################################
  //  JACK CONTROL
  //  ############################################################################################

  public void setJackHome() {
    jackOffset = jackLifterEncoder.getPosition();
    pitchOffset = ahrs.getPitch();
  }

  //Ejects the hatch from the robot autonomusly
  void ejectHatch(double approach){
 
    intakeEject.set(DoubleSolenoid.Value.kForward); // puts nubs out
    holdHeading(approach, .5);
    driveProfile(-7, 0, .4, approach, .5);      //backs up 7 inches
    intakeEject.set(DoubleSolenoid.Value.kReverse); //pull nubs in
  
  }

  //Ejects the hatch from the robot autonomusly
  void ejectCargo(double approach){
    // Roll cargo out and give time to release
    intakeMotor.set(ControlMode.PercentOutput, -1);
    holdHeading(approach, 1.0);
    driveProfile(-7, 0, .4, approach, 0.5);      //backs up 7 inches
   }


  // Provide an easy way to limit values to +/- limit
  public double clip(double raw, double limit) {
    if (raw > limit)
      return limit;
    else if (raw < -limit)
      return -limit;
    else
      return raw;
  }
}

