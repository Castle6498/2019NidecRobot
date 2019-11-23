package frc.robot;

import frc.lib.util.ConstantsBase;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants extends ConstantsBase {
    public static double kLooperDt = 0.005;
    
       
    // Drive
         //Talon
         public static final double regularTurnReduction=.85;

        public static final double kDriveSwivelReduction=1;


        public static final int kDriveLeftTalonID=9;
        public static final double kDriveLeftTalonP=.8;//.15
        public static final double kDriveLeftTalonI=0;
        public static final double kDriveLeftTalonD=0;
        

        public static final double kDriveLeftMaxSpeed=2350;
        public static final double kDriveLeftTalonF=(1.0*1023)/kDriveLeftMaxSpeed;


        public static final int kDriveRightTalonID=3;

       public static final double kDriveRightTalonP=kDriveLeftTalonP;
        public static final double kDriveRightTalonI=kDriveLeftTalonI;
        public static final double kDriveRightTalonD=kDriveLeftTalonD;
        
        public static final double kDriveRightMaxSpeed=2800;
        public static final double kDriveRightTalonF=(1.0*1023)/kDriveRightMaxSpeed;

        public static final double kDriveTicksPerInch=(4096)/(Math.PI*6);; //TODO: get wheel diam
        public static final double kDriveTolerance=5;

        //Victor
        public static final int kDriveLeftVictorID=2;
        public static final int kDriveRightVictorID=1;

        public static final int kDriveLeftTwoVictorID=12;
        public static final int kDriveRightTwoVictorID=4;

        public static final int kDriverShifterPort=3;
    //Ball Control Helper 
        //PickUp
            public static final double kLiftPickUpFloor = 0;
            public static final double kWristPickUpFloor = -125;
            public static final double kLiftPickUpLoadingStation = 20;
            public static final double kWristPickUpLoadingStation = 0;
 
        //Shoot Height
            public static final double kLiftShootCargoShip = 24;
            public static final double kWristShootCargoShip = -35;

            public static final double kLiftShootRocketOne = 6;
            public static final double kWristShootRocketOne = 0;

            public static final double kLiftShootRocketTwo = 34;
            public static final double kWristShootRocketTwo = 0;
        
        //Carry Height
            public static final double kLiftCarryLow = 0;
            public static final double kWristCarryLow = 0;

            public static final double kLiftCarryMiddle = 0;
            public static final double kWristCarryMiddle = 0;

        //Shoot
            public static final double kCarryPauseAfterShoot=1;

        //Auto Features
        public static final boolean kCarryAfterPickUp=true;
        public static final boolean kCarryAfterShoot=true;




    //Intake -----------------------------------------------------------
    
        //Talon
        public static final int kIntakeTalonID=5;

        public static final double kIntakePickUpSpeed=.8;
        public static final double kIntakeShootSpeed=-1;

        public static final double kIntakeBallRequiredTime=.55;
        public static final double kIntakeShootPause = 1; 

        public static final double kIntakeCurrentThreshold=1.5;

        //Photoeye
        public static final int kIntakeSensorPort = 0;
         
    //Wrist -----------------------------------------------------------
    
        //Talon
        public static final int kWristTalonID=8;
        public static final int kWristChildTalonID=11; 

        public static final double kWristTalonP=4;
        public static final double kWristTalonI=0;
        public static final double kWristTalonD=0;
        public static final double kWristTalonF=0;

        public static final double kWristTicksPerDeg=1024/360; 
        public static final double kWristSoftLimit=220; 
        public static final double kWristTolerance = .5;

        public static final double kWristMaxOutput=.6;



    //CLIMBING CONTROLS --------------------------------------------
        
        //Distances
            public static final double climbWristAngle=220;

            public static final double liftOffset=4;

            public static final double preClimbMiddleHeight=10;
            public static final double preClimbHighHeight=20.5;

            public static final double suspClimbOffset=1;

            public static final double liftClimbSetPoint=0;

            public static final double suspFloorPos = 1;

        //Cimb Lift
            public static final double kLiftClimbP=1;//1
            public static final double kLiftClimbI=0;
            public static final double kLiftClimbD=0;

            public static final double kLiftMaxSpeedClimb=1800;//1800
            public static final double kLiftClimbF=(1.0*1023)/kLiftMaxSpeedClimb;

            public static final double kLiftClimbVelocity=3;   
            public static final double kLiftClimbAcceleration=1; 

        //Climb Suspension
           // public static final int kSuspensionBackLiftTalonID=4;
                public static final double kSuspensionBackLiftTalonP=.3;//.1
                public static final double kSuspensionBackLiftTalonI=0;
                public static final double kSuspensionBackLiftTalonD=0;
                public static final double kSuspensionMaxSpeedClimb=96000;//96000;
                public static final double kSuspensionBackLiftF=(1.0*1023)/kSuspensionMaxSpeedClimb;

                public static final double kSuspensionVelocity=3;//10;   
                public static final double kSuspensionAcceleration=1; 

            public static final double kSuspensionLiftTicksPerInch=(1024*4)*4*16; //added hopefully from test pannel testing
            public static final int kSuspensionLiftSoftLimit=21; 
            public static final double kSuspensionLiftTolerance =.5;
      
        //Climb Wrist
            public static final double kWristClimbP=18;//10
            public static final double kWristClimbI=0;
            public static final double kWristClimbD=0;
            public static final double kWristClimbF=0;


    //Lift -----------------------------------------------------------
    
        //Talon
        public static final int kLiftTalonID=10;

        public static final double kLiftTalonP=1;//1.4;
        public static final double kLiftTalonI=0;
        public static final double kLiftTalonD=0;
        public static final double kLiftMaxSpeed=1900;//1800;
        public static final double kLiftTalonF=(1.0*1023)/kLiftMaxSpeed;

        public static final double kLiftVelocity=20.5;//32;   
        public static final double kLiftAcceleration=60;//20; 

        public static final double kLiftTicksPerInch=4096 /(1.4*Math.PI); 
        public static final double kLiftSoftLimit=34; 

        public static final double kLiftTolerance = .5;
        
    //PlateCenter -----------------------------------------------------------
    
        //Talon
        public static final int kPlateCenterTalonID=7;
            public static final double kPlateCenterTalonP=20;
            public static final double kPlateCenterTalonI=0;
            public static final double kPlateCenterTalonD=0;
            public static final double kPlateCenterTalonF=0;

            public static final double kPlateCenterTicksPerInch=1680 / (2.24*Math.PI); 
            public static final double kPlateCenterTalonSoftLimit=15; 
            public static final double kPlateCenterTalonTolerance = .01;

        //Pneumatics
        //public static final int kPlateCenterSuckSolenoidPort=2;
        public static final int[] kPlateCenterDeploySolenoidPort={0,4};
        public static final int kPlateCenterHardStopYeeYeeSolenoidPort=1;
        //public static final int[] kPlateVaccuumReleaseSolenoidPort={6,7};
        public static final int[] kPlateHardStopOneWaySolenoidPort={6,7};
        
                            //VacOn, Suck, Push, vacOff, Retract, FinalReset, Done
        public static final double[] kPlateCenterDeployPauses = {.1,.2,.2,1,.2,.2}; 

        public static final double kPlateCenterHardStopPause=.1;
        //Photoelectric                                            
        public static final int kPlateCenterEdgeFinderPort=1;

        public static final double kPlateCenterCenteringSpeed=.7; //.4 -----------------

        public static final int kPlateCenterWallDetectorPort = 3;

        //Ultrasonic
        public static final int[] kPlateCenterUltraPort={2,3};


        //LimeLight

        public static final double kLimeLightDistancetoTarget=25;
                                  
        public static final double kLimeLightDistanceFromCenter=2; //positive is movement to the left

        public static final boolean kLimeLightAutoDeploy=true;

        public static final double kLimeLightTargetPause = .25;




        public static final double kLimeTargetHeight=30;
        public static final double kLimeCameraAngleFromHorizontal=45;
        public static final double kLimeCameraHeight=43;  

        public static final double kLimeTriggerDistance=30;







            
 
    //LED -------------------------------------------------------------------------------

        public static final int[] kPlateLEDPorts = {4,5,6};
    

    

    @Override
    public String getFileLocation() {
        return "~/constants.txt";
    }

   
}
