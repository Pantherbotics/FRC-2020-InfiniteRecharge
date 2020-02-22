package frc.robot;

// Java imports
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

// I2C helpers
import edu.wpi.first.wpilibj.I2C;

// Other imports
import edu.wpi.first.wpilibj.Notifier;

/*
    Interface with raspberry pi over i2c connection
*/
public class Vision
{
    /*
        arbitrary i2c constants
    */
    
    // the address of raspberry pi on i2c port
    private final int kAddress = 0x69;

    // command registers to let PI know what we're tryna do
    private final byte kRegisterBegin = (byte) 0xa0;
    private final byte kRegisterGetData = (byte) 0xa1;
    private final byte kRegisterExposureTime = (byte) 0xa2;

    // seconds of exposure = kExposureTime / 10,000
    public byte kExposureTime = 10;

    /*
     * Helpers
     */
    private I2C mI2C;
    private Target mTarget;
    private Notifier updateLoop;

    /*
     * Variables
     */
    private int updateFailCount = 0;

    /*
     * Constructor
     */
    public Vision(I2C.Port physicalPort) {
        // initiate target
        this.mTarget = new Target();

        // open i2c port
        this.mI2C = new I2C(physicalPort, this.kAddress);
        
        // notify PI that we're listening
        this.mI2C.write(kRegisterBegin, (byte) 0xff);
        
        // create update loop
        this.updateLoop = new Notifier(() -> {
            this.update();
        });
        
        System.out.println("begin loop");
        this.updateLoop.startPeriodic(20f / 1000f);
        
    }
        
    /*
        updates the current target
        if the read operation works
    */
    private boolean updateTarget()
    {
        // create buffer
        System.out.println("loop");
        byte[] rawData = new byte[10];
        boolean aborted = this.mI2C.read(0, 10, rawData);

        // if the read is aborted
        if (aborted)
            return false;

        boolean found = false;
        int dataStart;
        for (dataStart = 0; dataStart < 6; dataStart++)
        {
            if (rawData[dataStart] == (byte)0x69)
            {
                found = true;
                break;
            }
        }

        if (!found)
            return false;
        
        byte[] byteData = {rawData[dataStart + 1], rawData[dataStart + 2], rawData[dataStart + 3], rawData[dataStart + 4]};
        
        // data is returned in little endian, convert to big (so my small brain can understand)
        int[] distance_raw = {byteData[1] & 0xff, byteData[0] & 0xff};
        int[] horizontal_raw = {byteData[3] & 0xff, byteData[2] & 0xff};
        int distance = (distance_raw[0] << 8) | distance_raw[1];
        int horizontal = (horizontal_raw[0] << 8) | horizontal_raw[1];
        // there is probably a better way of reading little endian unsigned shorts, but idc
        System.out.println("Sent: " + String.format("0x%02X ", byteData[0]) + ", "
                                    + String.format("0x%02X ", byteData[1]) + ", "
                                    + String.format("0x%02X ", byteData[2]) + ", "
                                    + String.format("0x%02X ", byteData[3]));
        
        // struct Target {
        //     unsigned short distance;
        //     unsigned short horizontal;
        // }
        this.mTarget.distance   = distance;
        this.mTarget.horizontal = horizontal;
        
        // target.exists = either is nonzero
        this.mTarget.exists = (this.mTarget.distance != 0) || (this.mTarget.horizontal != 0);
        System.out.println("Distance: " + this.mTarget.distance + ", Angle: " + this.mTarget.horizontal);
        return true;
    }
    
    /*
        updateTarget wrapper that handles errors
    */
    public void update()
    {
        try
        { // exit if update fails for any reason
            boolean success = this.updateTarget();
            assert success : "Failed to update target, but this err will be caught so who cares";
        }
        catch(Exception e)
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
