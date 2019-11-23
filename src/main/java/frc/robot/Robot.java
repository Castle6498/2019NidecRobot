package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.lib.util.CrashTracker;
import frc.lib.util.DriveSignal;
import frc.robot.CameraVision.CameraMode;
import frc.robot.CameraVision.LightMode;
import frc.robot.CameraVision.StreamMode;

import frc.robot.loops.Looper;
import frc.robot.state_machines.BallControlHelper;
//import frc.robot.state_machines.ClimbingHelper;
import frc.robot.state_machines.BallControlHelper.CarryHeight;
import frc.robot.state_machines.BallControlHelper.PickUpHeight;
import frc.robot.state_machines.BallControlHelper.ShootHeight;
//import frc.robot.state_machines.Superstructure;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.PlateCenter;
//import frc.robot.subsystems.Suspension;
import frc.robot.subsystems.Wrist;

/**
 * The main robot class, which instantiates all robot parts and helper classes
 * and initializes all loops. Some classes are already instantiated upon robot
 * startup; for those classes, the robot gets the instance as opposed to
 * creating a new object
 * 
 * After initializing all robot parts, the code sets up the autonomous and
 * teleoperated cycles and also code that runs periodically inside both
 * routines.
 * 
 * This is the nexus/converging point of the robot code and the best place to
 * start exploring.
 * 
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

    // Get subsystem instances
    private Drive mDrive = Drive.getInstance();
    private PlateCenter mPlate = PlateCenter.getInstance();
    private BallControlHelper mBall = BallControlHelper.getInstance();

    
    


    // Create subsystem manager
    private final SubsystemManager mSubsystemManager = new SubsystemManager(Arrays.asList(mPlate,mBall,
    Intake.getInstance(),Lift.getInstance(),Wrist.getInstance()));

    // Initialize other helper objects
    private ControlBoardInterface mControlBoard = ControlBoard.getInstance();

    private Looper mEnabledLooper = new Looper();

  
    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    public void zeroAllSensors() {
        mSubsystemManager.zeroSensors();
     
    }

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);

            //Here it is:
          //  AutoModeSelector.initAutoModeSelector();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
        zeroAllSensors();
    }

    @Override
    public void autonomousInit() {
       normalInitialization();
       mPlate.startedCenter(); //MEANS it started at the exact center (no need to home)
    }

    @Override
    public void autonomousPeriodic() {
       teleopPeriodic();
    }


    public void normalInitialization(){
        try {
            CrashTracker.logTeleopInit();

            // Start loopers
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mBall.setWantedState(BallControlHelper.SystemState.HOME);
            //mPlate.setWantedState(PlateCenter.SystemState.HOMING);
            //mClimbingHelper.setWantedState(ClimbingHelper.SystemState.HOME);
            
            CameraVision.setLedMode(LightMode.eOff);
            CameraVision.setCameraMode(CameraMode.eVision);
            CameraVision.setPipeline(1);
            CameraVision.setStreamMode(StreamMode.LimeMain);

           

           zeroAllSensors();
           
            
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
    /**
     * Initializes the robot for the beginning of teleop
     */
    @Override
    public void teleopInit() {
        normalInitialization();
            
        if(!mPlate.hasHomed())mPlate.setWantedState(PlateCenter.SystemState.HOMING);      
    }

    /**
     * This function is called periodically during operator control.
     * 
     * The code uses state machines to ensure that no matter what buttons the driver presses, the robot behaves in a
     * safe and consistent manner.
     * 
     * Based on driver input, the code sets a desired state for each subsystem. Each subsystem will constantly compare
     * its desired and actual states and act to bring the two closer.
     */
    @Override
    public void teleopPeriodic() {
        try {
            //double timestamp = Timer.getFPGATimestamp();
           
        
        //DRIVE ---------------------------------------------------------------------------------------

            mDrive.setOpenLoop(mControlBoard.getDriveSignal());
            mDrive.lowGear(mControlBoard.getLowGear());

            mDrive.drivePeriodic();

            mDrive.profileEnable(mControlBoard.enableMotionProfile());

            mDrive.holdMotionProile(mControlBoard.holdMotionProfile());
          
        //PLATE -------------------------------------------------------------------------------------

            if(mControlBoard.getHatchPanelAlignment()) mPlate.setWantedState(PlateCenter.SystemState.AUTOALIGNING);
            else if(mControlBoard.getHatchPanelCentering()) mPlate.setWantedState(PlateCenter.SystemState.CENTERING);
            else if(mControlBoard.getHatchPanelDeploy()) mPlate.setWantedState(PlateCenter.SystemState.DEPLOYINGPLATE);
            else if(mControlBoard.getPlateHome()) mPlate.setWantedState(PlateCenter.SystemState.HOMING);
        
            mPlate.deployHardStop(mControlBoard.getHatchHardStops());

            mPlate.jog(mControlBoard.getHatchPanelJog());

           // System.out.println("Plate Wall: "+mPlate.getWall());


        //BALL -----------------------------------------------------------------------------------
        
            PickUpHeight pickUpHeight = mControlBoard.getBallPickUp();
            ShootHeight shootHeight = mControlBoard.getBallShootPosition();
            CarryHeight carryHeight = mControlBoard.getCarryBall();

            if(pickUpHeight!=null) mBall.pickUp(pickUpHeight);
            else if(shootHeight!=null)mBall.shootPosition(shootHeight);
            else if (carryHeight!=null)mBall.carry(carryHeight);
            else if(mControlBoard.getBallShoot()) mBall.setWantedState(BallControlHelper.SystemState.SHOOT);
            else if(mControlBoard.getBallHome()) mBall.setWantedState(BallControlHelper.SystemState.HOME);
            
            mBall.jogLift(mControlBoard.getLiftJog());    
        
            mBall.jogWrist(mControlBoard.getWristJog());         
        
        //CLIMBING -----------------------------------------------------------------------------
            
         /*  if(mControlBoard.getSuspensionHome())mClimbingHelper.setWantedState(ClimbingHelper.SystemState.HOME);
            
            ClimbingHelper.PreClimbHeight h = mControlBoard.getPreClimbHeight();
            if(h!=null)mClimbingHelper.preClimb(h);

            if(mControlBoard.getClimbLift())mClimbingHelper.setWantedState(ClimbingHelper.SystemState.LIFT);
            else if(mControlBoard.getClimbStow())mClimbingHelper.setWantedState(ClimbingHelper.SystemState.STOW);
        
            mClimbingHelper.setFullBlast(mControlBoard.getClimbFullBlast());
        
            mClimbingHelper.setVerticalJog(mControlBoard.getClimbVerticleJog());
            mClimbingHelper.setHorizontalJog(mControlBoard.getClimbHorizontalJog());
*/

       
           


           allPeriodic();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();

            mDrive.stop();

            mEnabledLooper.stop();

            // Call stop on all our Subsystems.
            mSubsystemManager.stop();

           mDrive.setOpenLoop(DriveSignal.NEUTRAL);
           
           

            // If are tuning, dump map so far.
          /*  if (Constants.kIsShooterTuning) {
                for (Map.Entry<InterpolatingDouble, InterpolatingDouble> entry : mTuningFlywheelMap.entrySet()) {
                    System.out.println("{" +
                            entry.getKey().value + ", " + entry.getValue().value + "},");
                }
            }*/
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
      /*  final double kVoltageThreshold = 0.15;
        if (mCheckLightButton.getAverageVoltage() < kVoltageThreshold) {
            mLED.setLEDOn();
        } else {
            mLED.setLEDOff();
        }*/

        zeroAllSensors();
        allPeriodic();
    }

    @Override
    public void testInit() {
       
    }

    @Override
    public void testPeriodic() {
    }

    /**
     * Helper function that is called in all periodic functions
     */
    public void allPeriodic() {
        
        mSubsystemManager.outputToSmartDashboard();
        mSubsystemManager.writeToLog();
        mEnabledLooper.outputToSmartDashboard();
       
        
    }
}
