package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;

import frc.lib.util.drivers.Talon.CANTalonFactory;
import frc.robot.CameraVision;
import frc.robot.Constants;
import frc.robot.ControlBoard;
import frc.robot.ControlBoardInterface;
import frc.robot.CameraVision.CameraMode;
import frc.robot.CameraVision.LightMode;
import frc.robot.ControlBoardInterface.Controller;
import frc.robot.ControlBoardInterface.RumbleSide;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;

/**
 * The plateCenter subsystem consists of dynamo motors that are meant to send
 * one ball at a time into the shooter. There are ir sensors placed before and
 * after the plateCenter to sense blockage or emptiness of the hopper. The main
 * things this subsystem has to are feed fuel and unjam
 * 
 *
 */
public class PlateCenter extends Subsystem {
    
   
    private ControlBoardInterface mControlBoard = ControlBoard.getInstance();

    private static PlateCenter sInstance = null;

    public static PlateCenter getInstance() {
        if (sInstance == null) {
            sInstance = new PlateCenter();
        }
        return sInstance;
    }

    private TalonSRX mBeltTalon;
    private final Solenoid mHardStopYeeYeeSolenoid;
    private final DoubleSolenoid mDeploySolenoid, mOneWayHardStopSolenoid;
  
    DigitalInput mEdgeDetector, mWallDetector;

   // Ultrasonic mUltra;

    Compressor compressor;
 

    
    public PlateCenter() {
       

        //Configure Talon
            mBeltTalon = CANTalonFactory.createTalon(Constants.kPlateCenterTalonID,
            false, NeutralMode.Brake, FeedbackDevice.QuadEncoder, 0, false);

            mBeltTalon = CANTalonFactory.setupHardLimits(mBeltTalon, LimitSwitchSource.Deactivated,
            LimitSwitchNormal.Disabled, false, LimitSwitchSource.FeedbackConnector,
            LimitSwitchNormal.NormallyOpen, true);

            mBeltTalon = CANTalonFactory.setupSoftLimits(mBeltTalon, true, (int) Math.round(Constants.kPlateCenterTalonSoftLimit*Constants.kPlateCenterTicksPerInch),
            false, 0);

            mBeltTalon = CANTalonFactory.tuneLoops(mBeltTalon, 0, Constants.kPlateCenterTalonP,
            Constants.kPlateCenterTalonI, Constants.kPlateCenterTalonD, Constants.kPlateCenterTalonF);
  
        //Photoelectric
            mEdgeDetector = new DigitalInput(Constants.kPlateCenterEdgeFinderPort);

            mWallDetector = new DigitalInput(Constants.kPlateCenterWallDetectorPort);

        //ULTRA
          //  mUltra = new Ultrasonic(Constants.kPlateCenterUltraPort[0], Constants.kPlateCenterUltraPort[1]);
           
        //Pneumatics        
           
            mDeploySolenoid = new DoubleSolenoid(Constants.kPlateCenterDeploySolenoidPort[0],Constants.kPlateCenterDeploySolenoidPort[1]);
            mHardStopYeeYeeSolenoid = new Solenoid(Constants.kPlateCenterHardStopYeeYeeSolenoidPort);
            
            mOneWayHardStopSolenoid = new DoubleSolenoid(Constants.kPlateHardStopOneWaySolenoidPort[0],Constants.kPlateHardStopOneWaySolenoidPort[1]);


            compressor = new Compressor();

        System.out.println("Plate initialized");
    }

    private void setLimitClear(boolean e){
        mBeltTalon.configClearPositionOnLimitR(e,0);    
    }

    public enum SystemState {
        IDLE, 
        CENTERING, 
        AUTOALIGNING, 
        DEPLOYINGPLATE,
        HOMING
    }

    private SystemState mSystemState = SystemState.IDLE;
    private SystemState mWantedState = SystemState.IDLE;

    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            compressor.start();
            synchronized (PlateCenter.this) {
                System.out.println("Plate onStart");
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                mCurrentStateStartTime = timestamp;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (PlateCenter.this) {
                SystemState newState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case CENTERING:
                    newState = handleCentering();
                    break;                
                case AUTOALIGNING:
                    newState = handleAutoAligning(timestamp, mCurrentStateStartTime);
                    break;  
                case DEPLOYINGPLATE:
                    newState = handleDeployingPlate(timestamp, mCurrentStateStartTime);
                    break;  
                case HOMING:
                    newState = handleHoming(timestamp, mCurrentStateStartTime);
                    break;
                default:
                    newState = SystemState.IDLE;
                }
                if (newState != mSystemState) {
                    System.out.println("PlateCenter state " + mSystemState + " to " + newState + " "+timestamp);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
                if(hasHomed)positionUpdater();
                hardStopUpdater(timestamp);
            }
          //  System.out.println("Plate Loop");
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };  //LOOP SET ENDS HERE


    private SystemState defaultIdleTest(){
        if(mSystemState == mWantedState){
            mWantedState=SystemState.IDLE;
            return SystemState.IDLE; 
        }
        else return mWantedState;
    }
//double counter = 0;
    private SystemState handleIdle() {
        if(mStateChanged){
            stopMotor();
            //resetPistons();
           // counter=0;
        }
       //counter++;
       //if(counter%20==0)System.out.println("ultra: "+getDistance());
        return defaultIdleTest();
    }
    
    private boolean hasHomed = false;

    public void startedCenter(){
        mBeltTalon.setSelectedSensorPosition((int) Math.round((Constants.kPlateCenterTalonSoftLimit/2)*Constants.kPlateCenterTicksPerInch));
        hasHomed=true;
        setPosition(Constants.kPlateCenterTalonSoftLimit/2);

    }
    
    public boolean hasHomed(){
        return hasHomed;
    }

    private SystemState handleHoming(double now, double startedAt){
        if(mStateChanged){
            hasHomed=false;
            setLimitClear(true);
            mBeltTalon.set(ControlMode.PercentOutput,-.9);
            mBeltTalon.setSelectedSensorPosition(-500);
        }

        if((now-startedAt)>3) {
            System.out.println("plate reset triggered");
            stopMotor();
            return SystemState.IDLE;
        }
        


        if(!hasHomed&&mBeltTalon.getSelectedSensorPosition()==0){
            hasHomed=true;
            mTravelingSetPosition=.1;
            System.out.println("plate has Homed");
            setPosition(Constants.kPlateCenterTalonSoftLimit/2);
        }
       // System.out.println("Current pos: "+getPosition()+ " "+mWantedSetPosition);

       SystemState newState;

       if(hasHomed){
           if(atPosition()){
           newState= defaultIdleTest();
           }else{
               newState= mWantedState;
           }
       }else{
           newState= SystemState.HOMING;
       }

       if(newState!=SystemState.HOMING)setLimitClear(false);
       return newState;
   }

   double inchesToCenter;
   boolean plateCentered=false;

   enum CenteringState {FARLIMIT, SENSE, DONE};
   CenteringState centeringState = CenteringState.FARLIMIT;

   private SystemState handleCentering() {
       if(mStateChanged){
           System.out.println("Centering");
           setPosition(0);
           centeringState = CenteringState.FARLIMIT;
           plateCentered=false;
           mTravelingSetPosition=.1;
           //deployHardStop(false);
          // vacRelease(true);
          // suck(true);

          mControlBoard.setRumble(Controller.Operator, RumbleSide.both, .5);
       }    
       
     // System.out.println("Lidar: "+getEdge()+ " State: "+centeringState);

       switch(centeringState){
           case FARLIMIT:
           if(atPosition()){
               mBeltTalon.set(ControlMode.PercentOutput,Constants.kPlateCenterCenteringSpeed);
               centeringState=CenteringState.SENSE;
           }
           break;
           case SENSE:
               if(getEdge()){
                   stopMotor();
                   inchesToCenter = getPosition() - Constants.kPlateCenterTalonSoftLimit/2; 
                   System.out.println("centered succesfully, inches to center: "+inchesToCenter);
                   centeringState = CenteringState.DONE;
                   plateCentered=true;
               }else if(getPosition()>=Constants.kPlateCenterTalonSoftLimit&&centeringState!=CenteringState.FARLIMIT) {
                   System.out.println("center failed");
                   centeringState = CenteringState.DONE;
                   plateCentered=false;
               }
           break;
       }

       
       SystemState newState=SystemState.CENTERING;


       if(centeringState==CenteringState.DONE){
          // deployHardStop(true);
          mControlBoard.rumbleOff();
           newState= defaultIdleTest(); 
       } else if(mWantedState == SystemState.HOMING) newState=SystemState.HOMING;
     

       if(newState!=SystemState.CENTERING){
        stopMotor();
        centeringState=CenteringState.DONE;
        }

        return newState;
   }
    
   enum PlateDeployState {VacOn, Suck, Push, vacOff, Retract, FinalReset, Done}

   private PlateDeployState deployState = PlateDeployState.VacOn;
   private double lastStateStart=0;
   private double elapsedStateTime=0;

    private SystemState handleDeployingPlate(double now, double startStartedAt) {
        
        if (mStateChanged) {
           stopMotor();
           System.out.println("Deploying Plate");  
            deployState= PlateDeployState.VacOn;
            lastStateStart=now;
           mControlBoard.setRumble(Controller.Driver, RumbleSide.both, .5);
        }


         elapsedStateTime = now - lastStateStart;
        
       
        PlateDeployState newState=deployState;
        switch(deployState){
            case VacOn:
                //vacRelease(true);
                push(false);
                if(deployUpdate(Constants.kPlateCenterDeployPauses[0])) 
                    newState = PlateDeployState.Suck;
            break;
            case Suck: 
                //vacRelease(true);
                //suck(true);
                push(false);
                if(deployUpdate(Constants.kPlateCenterDeployPauses[1])) 
                    newState = PlateDeployState.Push;
            break;
            case Push: 
                //vacRelease(true);
               // suck(true);
                push(true);
                if(deployUpdate(Constants.kPlateCenterDeployPauses[2])) 
                    newState = PlateDeployState.vacOff;
            break;
            case vacOff: 
               // vacRelease(false);
               // suck(false);
                push(true);
                if(deployUpdate(Constants.kPlateCenterDeployPauses[3])) 
                    newState = PlateDeployState.Retract;
            break;
            case Retract:
               // vacRelease(false);
               // suck(false);
                push(false);
                if(deployUpdate(Constants.kPlateCenterDeployPauses[4])) 
                    newState = PlateDeployState.FinalReset;
            break;
            case FinalReset:
                //vacRelease(false);
                //suck(false);
                push(false);
              //  deployHardStop(false);
                setPosition(Constants.kPlateCenterTalonSoftLimit/2);
                if(deployUpdate(Constants.kPlateCenterDeployPauses[5])) 
                    newState = PlateDeployState.Done;
            break;
        }


        if(newState!=deployState){
            deployState=newState;
            lastStateStart=now;
            System.out.println("Plate Deploy State: "+deployState);
        }

        if(deployState==PlateDeployState.Done){
           mControlBoard.rumbleOff();
            return defaultIdleTest();
        }
        
        return SystemState.DEPLOYINGPLATE;
       
    }

    private boolean deployUpdate(double time){
        if(time<=elapsedStateTime) return true;
        else return false;
    }
double counter = 0;
    private SystemState handleAutoAligning(double now, double startStartedAt){
        boolean ready=false;

        if(plateCentered){
        
            if(mStateChanged){
               // CameraVision.setCameraMode(CameraMode.eVision);
                CameraVision.setPipeline(0);
               CameraVision.setLedMode(LightMode.eOn);
               counter=0;
               
            }


            //d = (h2-h1) / tan(a1+a2)
          //  double distance = (Constants.kLimeCameraHeight-Constants.kLimeTargetHeight)/
           //     Math.tan(Math.toRadians(Constants.kLimeCameraAngleFromHorizontal+CameraVision.getTy()));
           
           // double rumble = 1-(distance/Constants.kLimeTriggerDistance);
           
           //if(counter%15==0) mControlBoard.setRumble(rumble);
           counter++;
            
            //Find the offset of target from camera with 0 at middle
            double target = Constants.kLimeLightDistancetoTarget*Math.tan(Math.toRadians(CameraVision.getTx()));

            //Find set point for motor with 0 on the left 
            target = Constants.kPlateCenterTalonSoftLimit/2 - 
                    Constants.kLimeLightDistanceFromCenter - target +inchesToCenter+1;

            

            
                    ready = atPosition();
            

            if(!CameraVision.isTarget()||now-startStartedAt<=Constants.kLimeLightTargetPause)ready=false;
            else if(counter%1==0)setPosition(target);



           // System.out.println("Distantce: "+distance+" Target set point: "+target+ " ready: "+ready+" inches to center: "+inchesToCenter+" position: "+getPosition());


            SystemState newState=mWantedState;

            if(ready&&Constants.kLimeLightAutoDeploy&&getWall()){
                System.out.println("AUTO DEPLOY FOR YEETING OTHER TEAMS");
             newState = SystemState.DEPLOYINGPLATE;
             mWantedState=SystemState.DEPLOYINGPLATE;
            }


            if(newState != SystemState.AUTOALIGNING){
                CameraVision.setLedMode(LightMode.eOff);
               CameraVision.setPipeline(1);
                mControlBoard.rumbleOff();
               // CameraVision.setCameraMode(CameraMode.eDriver);
            }

           

            return newState;
        
        } else return defaultIdleTest();

    }
    
   
    //POSITION CONTROL
        private double mWantedSetPosition=.1;
        private double mTravelingSetPosition=0;

        public synchronized void setPosition(double pos){
            if(pos>=Constants.kPlateCenterTalonSoftLimit)pos=Constants.kPlateCenterTalonSoftLimit;
            else if(pos<0)pos=0;
            mWantedSetPosition=pos;
            
           // System.out.println("Set wanted pos to "+pos);
        }
        
        private boolean jog=false;
        public synchronized void jog(double amount){
            setPosition(mWantedSetPosition+=amount);
            jog=true;
      
        }

        public double getPosition(){
            return mBeltTalon.getSelectedSensorPosition()/Constants.kPlateCenterTicksPerInch;
        }

        public boolean atPosition(){
           if(Math.abs(mWantedSetPosition-getPosition())<=Constants.kPlateCenterTalonTolerance){
               return true;
           }else{
               return false;
           }
        }

        private void positionUpdater(){
           
            if(hasHomed&&mWantedSetPosition!=mTravelingSetPosition){

                mTravelingSetPosition=mWantedSetPosition;
                if(!jog)System.out.println("Wrist to "+mTravelingSetPosition);
                jog=false;
                mBeltTalon.set(ControlMode.Position, mTravelingSetPosition*Constants.kPlateCenterTicksPerInch);
            }
        }

    //Sensors
        public boolean getEdge(){
            return mEdgeDetector.get();
        }

        public boolean getWall(){
            return mWallDetector.get();
        }

       

    //Pneumatic Controls
        boolean pistonPrints = false;

       
        private void push(boolean p){
            if(p) mDeploySolenoid.set(DoubleSolenoid.Value.kForward);
            else mDeploySolenoid.set(DoubleSolenoid.Value.kReverse);
            if(pistonPrints)System.out.println("Push solenoid: "+p);
        }
        private void hardStop(boolean h){
          mHardStopYeeYeeSolenoid.set(h);
          if(pistonPrints)System.out.println("HardStop solenoid: "+h);
          
        }
        private void hardStopOneWay(boolean h){
            //h=!h;
            if(h) mOneWayHardStopSolenoid.set(DoubleSolenoid.Value.kForward);
            else mOneWayHardStopSolenoid.set(DoubleSolenoid.Value.kReverse);
            if(pistonPrints)System.out.println("Hard Stop One Way solenoid: "+h);
          }

          double updateStart=0;
        private void hardStopUpdater(double now){
            if(hardStopChanged){
                if(hardStop){
                    if(updateStart==0){
                        updateStart=now;
                        System.out.println("hard stop start: "+updateStart);
                    }
                    hardStop(true);
                    if(now-updateStart>=Constants.kPlateCenterHardStopPause){
                        hardStopOneWay(true);
                        hardStopChanged=false;
                        System.out.println("hard stop done at: "+now);
                        updateStart=0;
                    }


                }else{
                    hardStop(false);
                    hardStopOneWay(false);
                    hardStopChanged=false;
                }
            }
        }

        boolean hardStop = false;
        boolean hardStopChanged=false;
        public void deployHardStop(boolean d){
            if(d!=hardStop)hardStopChanged=true;
            
            hardStop=d;
        }

        private void resetPistons(){
           
           
            push(false);
           hardStop(false);
           hardStopOneWay(false);
        }

    //Boring Stuff

        private void stopMotor(){
            mBeltTalon.set(ControlMode.PercentOutput,0);
        }

        public synchronized void setWantedState(SystemState state) {
            mWantedState = state;
        }
 
        public boolean checkSystem() {
            System.out.println("Testing FEEDER.-----------------------------------");
            boolean failure=false;       
            return !failure;
        }

        @Override
        public void outputToSmartDashboard() {

        }

        @Override
        public void stop() {
            compressor.stop();
            setWantedState(SystemState.IDLE);
            resetPistons();
        }

        @Override
        public void zeroSensors() {

        }

        @Override
        public void registerEnabledLoops(Looper in) {
            in.register(mLoop);
        }

}