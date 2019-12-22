package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.lib.util.DriveSignal;
import frc.robot.CameraVision.StreamMode;
//import frc.robot.state_machines.ClimbingHelper;

//import frc.robot.state_machines.ClimbingHelper.PreClimbHeight;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Contains the button mappings for the competition control board. Like the
 * drive code, one instance of the ControlBoard object is created upon startup,
 * then other methods request the singleton ControlBoard instance. Implements
 * the ControlBoardInterface.
 * 
 * @see ControlBoardInterface.java
 */
public class ControlBoard implements ControlBoardInterface {
    private static ControlBoardInterface mInstance = null;
    

    public static ControlBoardInterface getInstance() {
        if (mInstance == null) {
            //if (kUseGamepad) {
               // mInstance = new GamepadControlBoard();
            //} else {
                mInstance = new ControlBoard();
            //}
        }
        return mInstance;
    }
    private final XboxController mDriver;
    

    protected ControlBoard() {
        mDriver =new XboxController(0);
    }

    //DRIVING ---------------------------------------------------------------------------
        @Override
        public double getThrottle() {
            driverArcadeDrive();
            return throttle;
        }
        

        @Override
        public double getTurn() {
            driverArcadeDrive();
            return turn;
        }

        boolean driveReduction=false;
        double driveReductionAmount = .7; //remember a higher number means less reduction
        
        
        
        double throttle=0;
        double turn=0;
        
        public void driverArcadeDrive() {
            throttle=0;
            turn=mDriver.getX(Hand.kLeft)*Constants.regularTurnReduction;
            if(mDriver.getTriggerAxis(Hand.kRight)>.05) {
                throttle=mDriver.getTriggerAxis(Hand.kRight);			
            }else if(mDriver.getTriggerAxis(Hand.kLeft)>.05) {
                throttle=-mDriver.getTriggerAxis(Hand.kLeft);
                turn=-turn;
            }else {
                throttle=0;
                turn=turn*Constants.kDriveSwivelReduction;
            }
            if(getDriveInverted()&&throttle!=0){
                //turn=turn;
                throttle=-throttle;
            }
            

            //System.out.println("turn: "+turn+" throttle: "+throttle);
        }

        @Override
        public DriveSignal getDriveSignal() {
            boolean squareInputs=true;
            double xSpeed;
            double zRotation;
            
            driverArcadeDrive();
            

            xSpeed=throttle;
        
            zRotation = turn;
            
        
            // Square the inputs (while preserving the sign) to increase fine control
            // while permitting full power.
            if (squareInputs) {
                xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
                zRotation = Math.copySign(zRotation * zRotation, zRotation);
            }
            
            

            double leftMotorOutput;
            double rightMotorOutput;
        
            double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
        
            if (xSpeed >= 0.0) {
                // First quadrant, else second quadrant
                if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
                } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
                }
            } else {
                // Third quadrant, else fourth quadrant
                if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
                } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
                }
            }
            double m_rightSideInvertMultiplier = -1.0;

            leftMotorOutput=(limit(leftMotorOutput) * 1);
            rightMotorOutput=(limit(rightMotorOutput) * 1 * m_rightSideInvertMultiplier);

            
        // System.out.println("Rot:"+turn+" xSpeed: "+xSpeed+" Left: "+leftMotorOutput+ " right: "+rightMotorOutput);
            return new DriveSignal(leftMotorOutput,rightMotorOutput,false);
            
        }

        boolean driveInverted=true;
        @Override
        public boolean getDriveInverted() {
            if(mDriver.getStickButtonReleased(Hand.kLeft)){
                driveInverted=!driveInverted;
                if(driveInverted)CameraVision.setStreamMode(StreamMode.LimeMain);
                else CameraVision.setStreamMode(StreamMode.USBMain);
            }
        
        
            return driveInverted;


        }

        protected double limit(double value) {
            if (value > 1.0) {
            return 1.0;
            }
            if (value < -1.0) {
            return -1.0;
            }
            return value;
        }

        @Override
        public boolean getStartIntake(){
            return mDriver.getAButton();
        }

        @Override
        public boolean getStopIntake(){
            return mDriver.getBButton();
        }
    
        @Override
        public boolean getUnfoldIntake(){
            return mDriver.getYButton();
        }
    
        @Override
        public double getHoodAngle(){
            return (180/Math.PI)*Math.atan2(-mDriver.getY(Hand.kRight),mDriver.getX(Hand.kRight));
        }
    
        @Override
        public boolean getUpdateHoodAngle(){
            return mDriver.getStickButton(Hand.kRight);
        }
   
        @Override
        public boolean getStartShoot() {
            return mDriver.getBumper(Hand.kRight);
        }

        @Override
        public boolean getStopShoot() {
            return mDriver.getBumper(Hand.kLeft);
        }

    //RUMBLE ------------------------------------------------------------------------------

        @Override
        public void setRumble(Controller c, RumbleSide type, double amount) {
            XboxController controller;
           controller=mDriver;

            switch(type){
                case left:
                    controller.setRumble(RumbleType.kLeftRumble,amount);              
                break;
                case right:
                   controller.setRumble(RumbleType.kRightRumble,amount);
                break;
                case both: 
                   controller.setRumble(RumbleType.kLeftRumble, amount);
                   controller.setRumble(RumbleType.kRightRumble,amount);
                break;
            }
            
        }

        @Override
        public void rumbleOff() {
            setRumble(Controller.Driver, RumbleSide.both, 0);
            setRumble(Controller.Operator, RumbleSide.both, 0);
        }

        @Override
        public void setRumble(double amount) {
            setRumble(Controller.Driver, RumbleSide.both, amount);
            setRumble(Controller.Operator, RumbleSide.both, amount);
        }
   

   

  

    
}
