package frc.robot;

// Java imports
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

// I2C helpers
import edu.wpi.first.wpilibj.I2C;

// Other imports
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Target;

/*
    Interface with raspberry pi over i2c connection
*/
public class Vision extends Subsystem
{
    /*
        arbitrary i2c constants
    */
    
    // the address of raspberry pi on i2c port
    private static final int kAddress = 0x69;
    
    // command registers to let PI know what we're tryna do
    private static final byte kRegisterBegin        = 0xa0;
    private static final byte kRegisterGetData      = 0xa1;
    private static final byte kRegisterExposureTime = 0xa2;
    
    // seconds of exposure = kExposureTime / 10,000
    public byte kExposureTime = 10;
    
    /*
        Helpers
    */
    private I2C mI2C;
    private Target mTarget;
    private Notifier updateLoop;
    
    /*
        Variables
    */
    private void updateFailCount = 0;
    
    /*
        Constructor
    */
    public Vision(I2C.Port physicalPort)
    {
        // initiate target
        this.mTarget = new Target();
        
        // open i2c port
        this.mI2C = new I2C(physicalPort, this.kAddress);
        
        // notify PI that we're listening
        this.mI2C.write(kRegisterBegin, (byte) 0xff);
        
        // create update loop
        this.updateLoop = new Notifier(this.update);
        this.updateLoop.startPeriodic(20f / 1000f);
    }
        
    /*
        updates the current target
        if the read operation works
    */
    private boolean updateTarget()
    {
        // create buffer
        ByteBuffer rawData = ByteBuffer.allocate(4);
        
        // if the read is aborted
        if (this.mI2C.read(this.kRegisterGetData, 4, rawData))
        {
            return false;
        }
        
        // data is returned in little endian
        rawData.order(ByteOrder.LITTLE_ENDIAN);
        
        // struct Target {
        //     unsigned short distance;
        //     unsigned short horizontal;
        // }
        this.mTarget.distance   = (int) (rawData.getShort(0) & 0xffff);
        this.mTarget.horizontal = (int) (rawData.getShort(2) & 0xffff);
        
        // target.exists = either is nonzero
        this.mTarget.exists = (this.mTarget.distance != 0) || (this.mTarget.horizontal != 0)
        
        return true;
    }
    
    /*
        updateTarget wrapper that handles errors
    */
    public void update()
    {
        boolean success = true;
        try
        { // exit if update fails for any reason
            assert this.updateTarget();
        }
        catch Exception e
        { // update has failed
            this.updateFailCount++;

            // if more than 5 consecutive fails...
            if (this.updateFailCount > 5)
            {
                System.out.println("Failed 5 times in a road... idk what to do");
                this.updateFailCount = 0;
            }
            
            // prevent success case code from running
            return;
        }
        
        // updated successfully
        this.updateFailCount = 0;
    }
    
    /*
        mTarget getter
    */
    public Target getTarget()
    {
        return this.mTarget;
    }
}




