package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants;
import frc.lib.util.DriveSignal;


/**
 * Class for controlling the Drive Base
 * 
 *
 */
public class Drive {

    private static Drive mInstance = new Drive();

    public static Drive getInstance() {
        return mInstance;
    }


    // Hardware
    private Spark mLeftMaster, mRightMaster;

    private Drive() {
        
        //Spark Initialization      
        
            mLeftMaster = new Spark(Constants.kDriveLeftSparkPort);
            mRightMaster = new Spark(Constants.kDriveRightSparkPort);
        
        setOpenLoop(DriveSignal.NEUTRAL);

    }

    /**
     * Update open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
    
        // Right side is reversed, but reverseOutput doesn't invert PercentVBus.
        // So set negative on the right master. TODO: Need to invert?
        mRightMaster.set(signal.getRight());
        mLeftMaster.set(signal.getLeft());
     
    }
    
   
}
