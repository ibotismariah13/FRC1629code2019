// ----------------------------------------------------------
// Copyright (c) 2019 GaCo FRC Team 1629
package frc.robot;

import frc.robot.Hardware;
import frc.robot.Targets;

import frc.robot.StartPostion;
import frc.robot.Hatch;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;



// #################################################
public class Robot extends SampleRobot {

  public Hardware m15 ; // Meshach 15
  private SendableChooser <StartPostion> startPosition = new SendableChooser<>();
  private SendableChooser <Hatch> firstStop = new SendableChooser<>();
  private SendableChooser <Hatch> secondStop = new SendableChooser<>();
  private final double CARGO_POS_DIS = 18 ;

  private StartPostion selStartPosition;
  private Hatch selFirstStop;
  private Hatch selSecondStop;

  private Timer timeout; // timer
  private boolean lastCollectHatch;
  private boolean intakeIsDown;
  

    //  ############################################################################################
    //  ############################################################################################
    @Override
    public void robotInit() {
      m15 = new Hardware();
      m15.init(this);
      m15.intakeExtend.set(Value.kReverse);

      timeout = new Timer();
      timeout.start();
      SmartDashboard.putBoolean("Prototype", false);
      SmartDashboard.putBoolean("Old Auto", false);

      lastCollectHatch    = false;  // Remember the last state of these buttons for press checking
      intakeIsDown  = false;

      //setup chosers
      //start position
      startPosition.setDefaultOption("None", StartPostion.NONE);
      startPosition.addOption("Left Side", StartPostion.LEFT_SIDE);
      startPosition.addOption("Left Center", StartPostion.LEFT_CENTER);
      startPosition.addOption("Right Center", StartPostion.RIGHT_CENTER);
      startPosition.addOption("Right Side", StartPostion.RIGHT_SIDE);
      SmartDashboard.putData("Start Position", startPosition);
      
  
      //first postion
      firstStop.setDefaultOption("None", Hatch.STOP);
      firstStop.addOption("Cargo Ship Front", Hatch.CARGO_FRONT);
      firstStop.addOption("Cargo Ship Near Side", Hatch.CARGO_SIDE_NEAR);
      firstStop.addOption("Cargo Ship Middle Side", Hatch.CARGO_SIDE_MIDDLE);
      firstStop.addOption("Cargo Ship Far Side", Hatch.CARGO_SIDE_FAR);
      firstStop.addOption("Rocket Near Low", Hatch.ROCKET_NEAR_LOW);
      firstStop.addOption("Rocket Near Middle", Hatch.ROCKET_NEAR_MIDDLE);
      firstStop.addOption("Rocket Near High", Hatch.ROCKET_NEAR_HIGH);
      firstStop.addOption("Rocket Far Low", Hatch.ROCKET_FAR_LOW);
      firstStop.addOption("Rocket Far Middle", Hatch.ROCKET_FAR_MIDDLE);
      firstStop.addOption("Rocket Far High", Hatch.ROCKET_FAR_HIGH);
  
      SmartDashboard.putData("First Stop", firstStop);
  
      //second posistion
      secondStop.setDefaultOption("None", Hatch.STOP);
      secondStop.addOption("Loader Ready", Hatch.LOADER);
      // secondStop.addOption("Cargo Ship Front", Hatch.CARGO_FRONT);
      secondStop.addOption("Cargo Ship Near Side", Hatch.CARGO_SIDE_NEAR);
      secondStop.addOption("Cargo Ship Middle Side", Hatch.CARGO_SIDE_MIDDLE);
      secondStop.addOption("Cargo Ship Far Side", Hatch.CARGO_SIDE_FAR);
      secondStop.addOption("Rocket Near Low", Hatch.ROCKET_NEAR_LOW);
      secondStop.addOption("Rocket Near Middle", Hatch.ROCKET_NEAR_MIDDLE);
      secondStop.addOption("Rocket Near High", Hatch.ROCKET_NEAR_HIGH);
      secondStop.addOption("Rocket Far Low", Hatch.ROCKET_FAR_LOW);
      secondStop.addOption("Rocket Far Middle", Hatch.ROCKET_FAR_MIDDLE);
      secondStop.addOption("Rocket Far High", Hatch.ROCKET_FAR_HIGH);
      SmartDashboard.putData("Second Stop", secondStop);
    }
  
//  ############################################################################################
  @Override
    protected void disabled() {
      double blinkTime = 0;
      boolean LEDBlink = false;
      while (isDisabled()) {
         if (timeout.get() > blinkTime) {
          m15.stream.setValue(2);
          LEDBlink = !LEDBlink;
          m15.led.setValue(LEDBlink ? 2: 1);
          blinkTime = timeout.get() + 1;
          m15.autoAlarm = (startPosition.getSelected() == StartPostion.NONE);
          m15.periodic();
        }
      }
    }
  
  //  ############################################################################################
  @Override
  public void operatorControl() {

    double power;

    //set the lift height to the lower level where we can track targets
    m15.setAutoSide(true);
    m15.led.setValue(0);
    m15.snapshot.setValue(1) ;  // Turn on snapshot
 
    m15.enableAutoLift();
    m15.setLiftHeight(m15.HEIGHT_ROCKET_HATCH1);
    

    // Ensure solenoids are set for correct positions
    m15.intakeExtend.set(Value.kForward);
    m15.intakeLower.set(Value.kReverse);
    m15.intakeEject.set(Value.kReverse);

    m15.periodic();
    m15.lockHeading();
    m15.setJackHome();

    while (isOperatorControl() && isEnabled()) {

      // Update all sensor, target and dashboard data
      m15.periodic();
  
      // Reset Gyro if Back and Start are both pressed
      if (m15.PB_ResetGyro) {
        m15.resetHeading();
      }

      // Try to detect which side of the field we are on, based on robot heading when starting to collecting hatchcover
      // A positive heading indicates that we are pointing to the right side of the field.
      if (m15.PB_CollectHatch && !lastCollectHatch )
        m15.setAutoSide(m15.normalizeHeading(m15.currentHeading) > 0);
      lastCollectHatch = m15.PB_CollectHatch;

      // Check for automated drive functions

      // Automated drive function to collect a hatch cover from loader
      if (m15.PB_CollectHatch) {
        m15.setLiftHeight(m15.HEIGHT_ROCKET_HATCH1);

        if (m15.targetLocked) {
          // drive to Loader station and check if we are close enough to grab hatch
          if (m15.driveVision(180)){
            m15.driveTillContact(20, .5, 180, 1.0);
            takeHatchFromLoader(true);

            // wait for the driver to let go of the controls.
            while(m15.PB_CollectHatch && isEnabled() ) {
              m15.stopRobot();
              m15.periodic();

              // The CoPilot can initiate a follow-on score with 
              if (m15.PB_AssistRocketHatchLow) {
                loadingToRocket1(0);
                waitForRelease();
              }
              else if (m15.PB_AssistRocketHatchMid) {
                loadingToRocket1(1);
                waitForRelease();
              }
              else if (m15.PB_AssistRocketHatchHigh) {
                loadingToRocket1(2);
                waitForRelease();
              }
            }
          }  // if drive vision
        }  // if target locked
        else {
          m15.driveJoystick();
        }
      }

      // Automated drive function to Score Hatch/Cargo
      else if(m15.PB_AutoScore){
        m15.setLiftHeight(m15.assistLiftTransitHeight);
        double approach;

        if ( m15.targetLocked) {
          m15.setAutoSide(true);

          // Determine the likely target approach angle
          approach = m15.findClosestTargetApproach();

          // Drive to target
          if (m15.driveVision(approach)) {
          // Provide plent of opportunities to stop the process by just letting go of trigger
            if (m15.PB_AutoScore) {
              if (m15.PB_AutoScore) {

                if ((m15.assistTargetType == TargetType.ROCKET_CARGO) || (m15.assistTargetType == TargetType.SHIP_CARGO))  {
                  if (m15.PB_AutoScore) {
                    m15.driveProfile(-4, 0, 0.4, approach, 1.5);  // release ball from contact
                    if (m15.PB_AutoScore) {
                      goToScoringHeight(m15.assistLiftScoreHeight);
                      if (m15.PB_AutoScore) {
                        m15.driveTillContact(20, .5, approach, 1.0);
                        if (m15.PB_AutoScore) {
                          m15.ejectCargo(approach);
                        }
                      }
                    }
                  }
                } // End If Cargo

                else {
                  if (m15.PB_AutoScore) {
                    goToScoringHeight(m15.assistLiftScoreHeight);
                    if (m15.PB_AutoScore) {
                      m15.driveTillContact(20, .5, approach,1);
                      if (m15.PB_AutoScore) {
                        m15.ejectHatch(approach);
                      }
                    }
                  }
                }  

                // Lower lift and Stay here till the driver releases the button
                if (m15.PB_AutoScore) {
                  m15.setLiftHeight(m15.HEIGHT_ROCKET_HATCH1);
                  waitForRelease();
                }
              }
            }
          }  // if drive vision
        } // if target locked
        else {
          m15.driveJoystick();
        }
      }
      else {
        m15.driveJoystick();
      }

      // now output power settings to drive motors
      m15.driveRobot();

      // ==========================   PILOT controls   ======================

      // Hatch cover ejector Pilot Right Bumper
      if (m15.PB_CollectorEject) {
        m15.intakeEject.set(Value.kForward);
      }
      else {
        m15.intakeEject.set(Value.kReverse);
      }

      // intake wheels.   Pilot A Y
      if (m15.PB_CollectorFeedIn)
        m15.intakeMotor.set(ControlMode.PercentOutput, 1);
      else if(m15.PB_CollectorFeedOut)
        m15.intakeMotor.set(ControlMode.PercentOutput, -1);
      else
        m15.intakeMotor.set(ControlMode.PercentOutput, 0.05);  // slight hold power

      // ==========================   Co-PILOT controls   ======================

      // intake extender.  Copilot 9-10
      if (m15.PB_CollectorExtend)
        m15.intakeExtend.set(Value.kForward);
      else if(m15.PB_CollectorRetract)
        m15.intakeExtend.set(Value.kReverse);
      
     
      // Intake lower for floor Copilot 8-4
      if (m15.PB_CollectorTiltDown) {
        m15.intakeLower.set(Value.kForward);
        intakeIsDown = true;
      }
      else if(m15.PB_CollectorTiltUp) {
        m15.intakeLower.set(Value.kReverse);
        intakeIsDown = false;
      }
      

      // Lift Height Control Copilot 1,2,3 5,6,7
      if (m15.PB_LiftRocketHatchLow)
        m15.setLiftHeight( m15.HEIGHT_ROCKET_HATCH1);
      else if(m15.PB_LiftRocketHatchMid)
        m15.setLiftHeight(m15.HEIGHT_ROCKET_HATCH2);
      else if(m15.PB_LiftRocketHatchHigh)
        m15.setLiftHeight(m15.HEIGHT_ROCKET_HATCH3);
      else if(m15.PB_LiftRocketCargoLow)
        m15.setLiftHeight(m15.HEIGHT_ROCKET_CARGO1);
      else if(m15.PB_LiftRocketCargoMid)
        m15.setLiftHeight(m15.HEIGHT_ROCKET_CARGO2);
      else if(m15.PB_LiftRocketCargoHigh)
        m15.setLiftHeight(m15.HEIGHT_ROCKET_CARGO3);
      else if(m15.PB_LiftLoaderShipCargo)
        m15.setLiftHeight(m15.HEIGHT_LOADERSHIP_CARGO);
      else if (m15.PB_EndGameClimbLow){
        m15.setLiftHeight(m15.HEIGHT_LOW_ENDGAME);
        m15.intakeLower.set(Value.kForward);
      }
        else if (m15.PB_EndGameClimbHigh)
      {
        m15.setLiftHeight(m15.HEIGHT_HIGH_ENDGAME);
        m15.intakeLower.set(Value.kForward);
      }
      
 
      // Jack lifter control.  Requires "Safety" press to lift above zero.
      //if ((m15.PB_EndGameRetractJack  &&  (m15.jackHeight <= 0)) || m15.PB_EndGameRetractJackOverride) {
      if (m15.PB_EndGameRetractJack  &&  (!m15.jackLimitUp.get() || m15.PB_EndGameRetractJackOverride)) {
        // drive jack up
        m15.jackLifter.set(1);
      } else if (m15.PB_EndGameExtendJack && ((m15.jackHeight > -22.0)|| m15.PB_EndGameRetractJackOverride)) {
        // drive jack down
        m15.jackLifter.set(-1);
      }
      else {
        m15.jackLifter.set(0);
      }
    

      // Control lift one of three modes:  auto, Manual or endGame (determined in periodic())
      if (m15.endgameActive) {
        // we are now in endgame because the jack has been lowered.
        // We are now running the lift as an auto-leveler
        m15.disableAutoLift();
 
        if (m15.currentPitch > 0.5) {
          power =  0.125; // Pitching up so we need to raise the lift so robot can go down
        }
        else if (m15.currentPitch < -0.5) {
          power = -0.2 + (m15.currentPitch * 0.17);  // was 0.15 Pitching down so we need to lower the lift so robot can go up
          power = m15.clip(power, 1);
        }
        else {
          power = -0.25; 
        }

        m15.setLiftPower(power);
      }
      else {
        // we are in Teleop period 

        if (m15.PB_LiftUseSetpoint) {
          // We are running normal lift control
          m15.enableAutoLift();
        }
        else {
          // We are running manual lift control
          m15.disableAutoLift();

          if (m15.PB_LiftManualUp && !m15.liftLimitTop.get() ) {
            power = (m15.liftHeight < 65 ) ? 1.0 : 0.50;
          }
          else if (m15.PB_LiftManualDown && !m15.liftLimitBottom.get()) {
            if (m15.liftHeight > 7 )
              power = -0.4 ;
            else if (intakeIsDown)
              power = -0.2;
            else 
              power = 0.05;
          }
          else if (m15.liftLimitBottom.get()) {
            power = 0;
          }
          else {
            power = m15.HOLDING_POWER;
          }

          m15.setLiftPower(power);
        }
      }
    }

    // End of teleop...
    m15.disableAutoLift();
    m15.stopRobot();
    m15.snapshot.setValue(0) ;  // Turn off snapshot
   
  }

  private void waitForRelease() {
    while((m15.PB_CollectHatch || m15.PB_AutoScore) && isEnabled() ) {
      m15.stopRobot();
      m15.periodic();
    }
  }

  //  ############################################################################################
  @Override
  public void autonomous() {

    m15.snapshot.setValue(1) ;  // Turn on snapshot

    // Initialize the Gro heading to zero
    m15.resetHeading();

    //set the lift height to the lower level where we can track targets
    m15.led.setValue(0);
    m15.enableAutoLift();
    m15.setLiftHeight(m15.HEIGHT_ROCKET_HATCH1);
    m15.intakeExtend.set(Value.kForward);  // try doing this here.  If it rubs, may need to be mover to later in the sequence.

    // Get Auto mode settings
    selStartPosition = startPosition.getSelected();
    selFirstStop = firstStop.getSelected();
    selSecondStop = secondStop.getSelected();

    //Set auto starting SIDE.  This way the driving methods can automatcially flip the directions for the left side.
    m15.setAutoSide((selStartPosition == StartPostion.RIGHT_CENTER) || (selStartPosition == StartPostion.RIGHT_SIDE));

    // Run the auto sequence if the First Position is NOT STOP
    if (selFirstStop != Hatch.STOP){
      runAutoLogic();     
    }

    // Provide time for lift to settle.
    m15.holdHeading(m15.getHeading() * m15.flipFactor, 1.0);
    m15.stopRobot();
    m15.periodic();  // update PIDs

    m15.snapshot.setValue(0) ;  // Turn off snapshot
   
  }
  
  //  ############################################################################################
  //  AUTO functions  Sequencer
  //  ############################################################################################
  private void runAutoLogic(){

    // determine whether we are placing a second hatch.
    boolean doubleHatch = (selSecondStop != Hatch.STOP);

    // If we are starting in the center, run the first portion of a front Run
    if ((selStartPosition == StartPostion.LEFT_CENTER) || (selStartPosition == StartPostion.RIGHT_CENTER)){
      // Just run the basic drive straight.
      habToCargoFront(doubleHatch);
    }
    else {
      // Run the first portion of each Side run.
      switch (selFirstStop) {
        case CARGO_FRONT:
         habToCargoFront(doubleHatch);
          break;

        case CARGO_SIDE_NEAR:
          habToCargoSide(doubleHatch, 0);
          break;

        case CARGO_SIDE_MIDDLE:
          habToCargoSide(doubleHatch, 1);
          break;

        case CARGO_SIDE_FAR:
          habToCargoSide(doubleHatch, 2);
          break;

        case ROCKET_NEAR_LOW:
          habToRocket1(doubleHatch, 0);
          break;

        case ROCKET_NEAR_MIDDLE:
          habToRocket1(doubleHatch, 1);
          break;

        case ROCKET_NEAR_HIGH:
          habToRocket1(doubleHatch, 2);
          break;

        case ROCKET_FAR_LOW:
          habToRocket3(doubleHatch, 0);
          break;

        case ROCKET_FAR_MIDDLE:
          habToRocket3(doubleHatch, 1);
          break;

        case ROCKET_FAR_HIGH:
          habToRocket3(doubleHatch, 2);
          break;

        default:
          // Do nothing
          break;
      }  // end switch
    }
    // Run the second portion of each run.
    if (doubleHatch) {
 
      //lift hatch cover out of loader
      takeHatchFromLoader();

      switch (selSecondStop) {
        //case CARGO_FRONT:
        //  loadingToCargoFront();
        //  break;

        case CARGO_SIDE_NEAR:
          loadingToCargoSide(0);
          break;

        case CARGO_SIDE_MIDDLE:
          loadingToCargoSide(1);
          break;

        case CARGO_SIDE_FAR:
          loadingToCargoSide(2);
          break;

        case ROCKET_NEAR_LOW:
          loadingToRocket1(0);
          break;

        case ROCKET_NEAR_MIDDLE:
          loadingToRocket1(1);
          break;

        case ROCKET_NEAR_HIGH:
          loadingToRocket1(2);
          break;

        case ROCKET_FAR_LOW:
          loadingToRocket3(0);
          break;

        case ROCKET_FAR_MIDDLE:
          loadingToRocket3(1);
          break;

        case ROCKET_FAR_HIGH:
          loadingToRocket3(2);
          break; 

        case LOADER:
        default:
          // Do nothing
          break;
      }  // End switch
    }  // End if doubleHatch
  }

  //  ############################################################################################
  //  AUTO functions  FIRST HATCH
  //  ############################################################################################

  void habToCargoFront(boolean doubleHatch){
    //drive straight
    m15.driveProfile(62, 0, .5, 0, 3);
    m15.driveToTarget(Targets.RC1, 60, 0, 0.5 ,0, 14, 3);
    m15.driveTillContact(20, .5, 0, 1);   // was 0.4
    m15.ejectHatch(0);

    // Go to loader if we are doing a double hatch
    if (doubleHatch) {
      // travel to loaded here
      if (m15.oldAuto) {
        m15.turnToHeading(120, 1.5);
        m15.driveProfile(90, 0, .6, 120, 4);
        m15.driveToTarget(Targets.RL1, 100, 0, 0.4 ,160, 14, 4);
        m15.driveTillContact(20, .4, 180, 1);
      }
      else {
        m15.driveVector(150, 0.8, 140, 185, 3.5);
        m15.driveToTarget(Targets.RL1, 40, 0, 0.5 ,180, 14, 1.5); // was s sec
        m15.driveTillContact(20, .5, 180, 1); // was 1 sec
      }
    }
  }

  //to front hab when on left or right side
  void habToCargoSide(boolean doubleHatch, double location){

    if (m15.oldAuto) {
      // Go to correct port on cargo ship
      m15.driveProfile(80, 0, .8, 0, 5);
      m15.driveProfile((82 + (location * CARGO_POS_DIS)), 50, .7, 0, 3.5); // was 100
      m15.turnToHeading(-90, 1);
      m15.driveToTarget(Targets.RC2, 36, 0, .4, -90, 14, 3);
      m15.driveTillContact(20, 0.4, -90, 1) ;
      m15.ejectHatch(-90);

      // Go to loader if we are doing a double hatch
      if (doubleHatch) {
        m15.driveProfile(-24, -(location * CARGO_POS_DIS), .5, -90, 3);
        m15.turnToHeading(-180, 2);

        m15.driveProfile(100, -60, .7, -180, 3.5); //was .6
        m15.driveToTarget(Targets.RL1, 124, 0 , .7, -180, 14, 10);
        m15.driveTillContact(20, 0.4, -180, 1);
      }
    }
    else {
      // Go to correct port on cargo ship
      m15.driveProfile(94  , 0, .8, 0, 5);
      m15.driveVector(100, .8, 30, -90, 4);
      if ( location > 0 )
        m15.driveVector((location * CARGO_POS_DIS), .8, 0, -90, 4);
        
      m15.driveToTarget(Targets.RC2, 36, 0, .6, -90, 14, 3);
      m15.driveTillContact(20, 0.4, -90, 1) ;
      m15.ejectHatch(-90);
 
      // Go to loader if we are doing a double hatch
      if (doubleHatch) {
        m15.driveVector(160, .8, 150, 180, 5);
        m15.driveToTarget(Targets.RL1, 120 + (location * CARGO_POS_DIS), 0 , .8, 180, 14, 10);
        m15.driveTillContact(20, 0.4, -180, 1);
       }
        
    }
  }

  void habToRocket1(boolean doubleHatch, int height){

    //drive to rocket 1
    m15.driveProfile(44, 0, .8, 0, 4); //drive forward

    //m15.turnToHeading(65, 2); //turn towards rocket
    //m15.driveToTarget(Target.RR1, 87 , 0, .5 , 65, 14, 3); 
  
    m15.turnToHeading(30, 2); //turn towards rocket
    m15.driveProfile(44, 34, .7, 30, 4); //drive forward
    m15.driveToTarget(Targets.RR1, 47 , 0, .6 , 30, 14, 3); 
  
    // Lift to the correct height
    goToRocketHatchHeight(height);

    //  Make contact, then eject hatch and lower lift
    m15.driveTillContact(20, .5, 30, 3); 
    m15.ejectHatch(30);
    m15.setLiftHeight(m15.HEIGHT_ROCKET_HATCH1);

    // Go to loader if we are doing a double hatch
    if (doubleHatch) {
      m15.driveProfile(-24, 0, .8, 45, 2);  ///    45 vs 30 degrees???
      m15.turnToHeading(180, 1.5);
      m15.driveToTarget(Targets.RL1, 110, -24, .7, 180, 14, 5);
      m15.driveTillContact(20, .5, 180, 1.5);
    }
  }

  void habToRocket3(boolean doubleHatch, int height){

    //drive to rocket 3
    m15.driveProfile(80, 0, .8, 0, 5);
    m15.turnToHeading(30, 1);
    m15.driveProfile(110, 0, .8, 30, 5);
    m15.turnToHeading(150, 2);
    m15.driveToTarget(Targets.RR3, 24, 0, .3, 150, 14, 2);
    goToRocketHatchHeight(height);
    
    //  Make contact, then eject hatch and lower lift
    m15.driveTillContact(16, .4, 150, 1); 
    m15.ejectHatch(150);
    m15.setLiftHeight(m15.HEIGHT_ROCKET_HATCH1);

    // Go to loader if we are doing a double hatch
    if (doubleHatch) {
      m15.driveProfile(0, 36, .6, 150, 2);
      m15.turnToHeading(170, 2);
      m15.driveProfile(80, 0, .8, 170, 3);
      m15.driveToTarget(Targets.RL1, 80, -48, .6, 170, 14, 8);
      m15.driveTillContact(20, .4, 180, 1);
    }
  }

 
  //  ########################################  ####################################################
  //  AUTO functions  SECOND HATCH
  //  ############################################################################################

  void loadingToCargoSide(int location){

    //drive from right loading to right cargo ship
    if (m15.oldAuto) {
      m15.driveProfile(-110, 40, .8, -180, 10);
      m15.driveProfile((-115 - (location*CARGO_POS_DIS)), 40, .6, -180, 4); // was 115
      m15.turnToHeading(-90, 2);
    }
    else {
      m15.driveVector(142, 0.8, -40, 180, 4);  // was 30 deg
      m15.driveVector(96 + (location*CARGO_POS_DIS), 0.7, -10, -90, 5);
    }

    m15.driveToTarget(Targets.RC2, 48, 0 , .6, -90, 14, 3);  // was 0.5
    m15.driveTillContact(20, 0.4, -90, 2);
    m15.ejectHatch(-90);
  }

  /*
  void loadingToCargoFront(){

    //drive from right loading to right cargo ship
    m15.driveProfile(-110, 40, .8, -180, 10);
    m15.turnToHeading(0, 2);
    m15.driveProfile(0, -100, .8, 0, 10);
    m15.driveToTarget(Target.RC1, 32, 0 , .5, 0, 14, 3);
    m15.driveTillContact(20, 0.4, 0, 2);
    m15.ejectHatch(0);
  }
  */

  void loadingToRocket1(int height){

    //drive to rocket 1
    m15.driveProfile(-118, 40, .8, -180, 5);
    m15.turnToHeading(-330, 1.2);
    m15.driveToTarget(Targets.RR1, 53, 0, .4, -330, 14, 4);

    // Lift to the correct height
    goToRocketHatchHeight(height);
    //Make contact, eject hatch then lower lift
    m15.driveTillContact(20, 0.3, -330, 2);
    m15.ejectHatch(-330);
    m15.setLiftHeight(m15.HEIGHT_ROCKET_HATCH1);
  }

  void loadingToRocket3(int height){

    //drive to rocket 3
    m15.driveProfile(-260, 50, .8, 180, 8);
    m15.driveProfile(0, -24, .4, 180, 3);
    m15.turnToHeading(150, 2);
    m15.driveToTarget(Targets.RR3, 48, -15, .6, 150, 14, 3);

    // Lift to the correct height
    goToRocketHatchHeight(height);

    //Make contact, eject hatch then lower lift
    m15.driveTillContact(20,  .5, 150, 2);
    m15.ejectHatch(150);
    m15.setLiftHeight(m15.HEIGHT_ROCKET_HATCH1);
  }

  //  ############################################################################################
  //  AUTO functions helper methods
  //  ############################################################################################

  void goToRocketHatchHeight(int height){

    //  raise to correct hatch height
    if (height == 0) {
      goToScoringHeight(m15.HEIGHT_ROCKET_HATCH1);
    }
    else if (height == 1){
        goToScoringHeight(m15.HEIGHT_ROCKET_HATCH2);
    }
    else {
      goToScoringHeight(m15.HEIGHT_ROCKET_HATCH3);
    }
  }

  void goToScoringHeight(double scoreHeight){

    double endTime = timeout.get() + 0.25 + (Math.abs(scoreHeight - m15.liftHeight) / m15.LIFT_INCHES_PER_SECOND );
    m15.setLiftHeight(scoreHeight);

    //wait for lift to get in postion (but not too long)
    m15.periodic();
    while (isEnabled() && !m15.liftInPosition  && (timeout.get() < endTime)){
      m15.periodic();
    }
  }

  void takeHatchFromLoader() {
    // Lift hatch from loader and back out
    takeHatchFromLoader(false);
  }

  void takeHatchFromLoader(boolean push) {
    // Lift hatch from loader and back out
    m15.setLiftHeight(m15.HEIGHT_LOADER_HATCH + 14);
    m15.holdPosition(0.6);
    if (push)
      m15.boostGrab(0.3, 0.1);
    m15.driveProfile(-6, 0, .4 , -180, .5);
    m15.setLiftHeight(m15.HEIGHT_ROCKET_HATCH1);
  }
}
