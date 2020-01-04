package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.lib.util.CrashTracker;
import frc.lib.util.DriveSignal;
import frc.robot.CameraVision.CameraMode;
import frc.robot.CameraVision.LightMode;
import frc.robot.CameraVision.StreamMode;

import frc.robot.loops.Looper;

//import frc.robot.state_machines.ClimbingHelper;

//import frc.robot.state_machines.Superstructure;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

//import frc.robot.subsystems.Lift;
//import frc.robot.subsystems.PlateCenter;
//import frc.robot.subsystems.Suspension;
//import frc.robot.subsystems.Wrist;

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
    private Shooter mShooter = Shooter.getInstance();
    private Intake mIntake = Intake.getInstance();
    
    


    // Create subsystem manager
    private final SubsystemManager mSubsystemManager = new SubsystemManager(Arrays.asList(mShooter,Hood.getInstance(),
    mIntake,Feeder.getInstance(),Flywheel.getInstance()));

    // Initialize other helper objects
    private ControlBoardInterface mControlBoard = ControlBoard.getInstance();

    private Looper mEnabledLooper = new Looper();

  
    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    public void zeroAllSensors() {
        //mSubsystemManager.zeroSensors();
     
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
       //mPlate.startedCenter(); //MEANS it started at the exact center (no need to home)
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
            
       // if(!mPlate.hasHomed())mPlate.setWantedState(PlateCenter.SystemState.HOMING);      
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
          

        if(mControlBoard.getStartIntake())mIntake.setWantedState(Intake.WantedState.PICKUP);
        else if(mControlBoard.getStopIntake())mIntake.setWantedState(Intake.WantedState.IDLE);
        else if(mControlBoard.getUnfoldIntake())mIntake.setWantedState(Intake.WantedState.UNFOLD);

       if(mControlBoard.getUpdateHoodAngle()) mShooter.setHoodAngle(mControlBoard.getHoodAngle());
           
        if(mControlBoard.getStartShoot())mShooter.setWantedState(Shooter.WantedState.SHOOT);
        else if(mControlBoard.getStopShoot())mShooter.setWantedState(Shooter.WantedState.IDLE);

        if(mControlBoard.getIntakeActuateEnable()) mIntake.setActuator(mControlBoard.getIntakeActuateSpeed());
        else mIntake.setActuator(0);

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

           // mDrive.stop();

            mEnabledLooper.stop();

            // Call stop on all our Subsystems.
           // mSubsystemManager.stop();

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
        
      //  mSubsystemManager.outputToSmartDashboard();
      //  mSubsystemManager.writeToLog();
        mEnabledLooper.outputToSmartDashboard();
       
        
    }
}
