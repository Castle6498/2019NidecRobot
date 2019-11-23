package frc.robot.subsystems;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.motion_profile.GeneratedMotionProfileLeft;
import frc.robot.motion_profile.GeneratedMotionProfileRight;
import frc.robot.motion_profile.MotionProfileHelper;
import frc.lib.util.DriveSignal;
import frc.lib.util.drivers.Talon.CANTalonFactory;

/**
 * 
 * 
 *
 */
public class Drive {

    private static Drive mInstance = new Drive();

    
    //private static final int kHighGearVelocityControlSlot = 1;

    public static Drive getInstance() {
        return mInstance;
    }



    // Hardware
   // private final CANTalon mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
    private final Solenoid mShifter;
    
    private TalonSRX mLeftMaster, mRightMaster, mLeftTwoSlave, mRightTwoSlave;
    private VictorSPX mLeftSlave, mRightSlave ;
   
    MotionProfileHelper _leftProfileController;
    MotionProfileHelper _rightProfileController;
    
    //private final NavX mNavXBoard;

    private Drive() {
        
         //Talon Initialization 
            mLeftMaster = CANTalonFactory.createTalon(Constants.kDriveLeftTalonID, 
            true, NeutralMode.Brake, FeedbackDevice.QuadEncoder, 0, false);
    
            mLeftMaster = CANTalonFactory.setupHardLimits(mLeftMaster, LimitSwitchSource.Deactivated,
            LimitSwitchNormal.Disabled, false, LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled,false);
            
            mLeftMaster = CANTalonFactory.setupSoftLimits(mLeftMaster, false, 0, false, 0);
            
            mLeftMaster = CANTalonFactory.tuneLoops(mLeftMaster, 0, Constants.kDriveLeftTalonP,
            Constants.kDriveLeftTalonI, Constants.kDriveLeftTalonD, Constants.kDriveLeftTalonF);

            mRightMaster = CANTalonFactory.createTalon(Constants.kDriveRightTalonID, 
            true, NeutralMode.Brake, FeedbackDevice.QuadEncoder, 0, false);
    
            mRightMaster = CANTalonFactory.setupHardLimits(mRightMaster, LimitSwitchSource.Deactivated,
            LimitSwitchNormal.Disabled, false, LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled,false);
            
            mRightMaster = CANTalonFactory.setupSoftLimits(mRightMaster, false, 0, false, 0);
            
            mRightMaster = CANTalonFactory.tuneLoops(mRightMaster, 0, Constants.kDriveRightTalonP,
            Constants.kDriveRightTalonI, Constants.kDriveRightTalonD, Constants.kDriveRightTalonF);
        
        
        
        //Victor Initialization
            mLeftSlave = new VictorSPX(Constants.kDriveLeftVictorID);
            mLeftSlave.setInverted(true);
            mLeftSlave.follow(mLeftMaster);
            
            mRightSlave = new VictorSPX(Constants.kDriveRightVictorID);
            mLeftSlave.setInverted(true);
            mRightSlave.follow(mRightMaster);

            mLeftTwoSlave = new TalonSRX(Constants.kDriveLeftTwoVictorID);
            mLeftTwoSlave.setInverted(true);
            mLeftTwoSlave.follow(mLeftMaster);

            mRightTwoSlave = new TalonSRX(Constants.kDriveRightTwoVictorID);
            mRightTwoSlave.setInverted(true);
            mRightTwoSlave.follow(mRightMaster);

            mShifter=new Solenoid(Constants.kDriverShifterPort);
        
       
        setOpenLoop(DriveSignal.NEUTRAL);

        
        //mNavXBoard = new NavX(SPI.Port.kMXP);


         _leftProfileController = new MotionProfileHelper(mLeftMaster,GeneratedMotionProfileLeft.Points, GeneratedMotionProfileLeft.kNumPoints,true);
         _rightProfileController = new MotionProfileHelper(mRightMaster,GeneratedMotionProfileRight.Points, GeneratedMotionProfileRight.kNumPoints,false);
      
    }


   

   public void drivePeriodic(){


    _leftProfileController.control();
    _rightProfileController.control();

    if(profileEnable){
        SetValueMotionProfile setOutputL = _leftProfileController.getSetValue();
        SetValueMotionProfile setOutputR = _rightProfileController.getSetValue();

        if(hold) {
            setOutputL=SetValueMotionProfile.Hold;
            setOutputR=SetValueMotionProfile.Hold;
        }
					
      mLeftMaster.set(ControlMode.MotionProfile,setOutputL.value);

      
      mRightMaster.set(ControlMode.MotionProfile,setOutputR.value);
    }

   }

   boolean profileEnable=false;

   
   public void profileEnable(boolean e){
       if(e!=profileEnable&&e){ //if it changed to true
        System.out.println("starting profile");
           startMotionProfile();
       }else if(e!=profileEnable&&!e) { //if it changed to false
            System.out.println("stopping profile");
           stop();
       }
       profileEnable = e;

       if(!e) {
        _leftProfileController.reset();
        _rightProfileController.reset();
       }
       

   }
boolean hold=false;
   public void holdMotionProile(boolean h){
       //_leftProfileController.hold(hold);
       //_rightProfileController.hold(hold);
       hold = h;
   }

   public void startMotionProfile(){
    _leftProfileController = new MotionProfileHelper(mLeftMaster,GeneratedMotionProfileRight.Points, GeneratedMotionProfileRight.kNumPoints,true);
    _rightProfileController = new MotionProfileHelper(mRightMaster,GeneratedMotionProfileLeft.Points, GeneratedMotionProfileLeft.kNumPoints,false);
      

    _leftProfileController.startMotionProfile();
    _rightProfileController.startMotionProfile();
   }

   public void stop(){
       profileEnable=false;
       setOpenLoop(DriveSignal.NEUTRAL);
       _leftProfileController.reset();
       _rightProfileController.reset();
   }

    public void lowGear(boolean f){
        mShifter.set(f);
    }
    /**
     * Update open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
     if(!profileEnable){
        setBrakeMode(signal.getBrakeMode());
        // Right side is reversed, but reverseOutput doesn't invert PercentVBus.
        // So set negative on the right master.
        mRightMaster.set(ControlMode.PercentOutput,signal.getRight());
        mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
     }
    }

  
    private boolean mIsBrakeMode=false;
  
    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            if(on){
            mRightMaster.setNeutralMode(NeutralMode.Brake);
            mRightSlave.setNeutralMode(NeutralMode.Brake);
            mLeftMaster.setNeutralMode(NeutralMode.Brake);
            mLeftSlave.setNeutralMode(NeutralMode.Brake);
            }else{
            mRightMaster.setNeutralMode(NeutralMode.Coast);
            mRightSlave.setNeutralMode(NeutralMode.Coast);
            mLeftMaster.setNeutralMode(NeutralMode.Coast);
            mLeftSlave.setNeutralMode(NeutralMode.Coast);
            }
        }
    }

   

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0);
        mRightMaster.setSelectedSensorPosition(0);
     }

   

   

    private synchronized double inchesToTicks(double inches){
        return inches*Constants.kDriveTicksPerInch;
    }

   

   
}
