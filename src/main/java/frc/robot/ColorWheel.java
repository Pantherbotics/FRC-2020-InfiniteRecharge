package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;

public class ColorWheel
{
    private ColorSensorV3 sensor;

    // if only java's enums weren't terrible
    public int NONE   = -1;
    public int RED    =  0;
    public int GREEN  =  1;
    public int BLUE   =  2;
    public int YELLOW =  3;

    // tune me!
    public int MAX_PROXIMITY = 10;

    // for counting rotations
    private int lastColor = -1;
    private int colorChanges;
    private Notifier counter;
    
    public ColorWheel(I2C.Port port)
    {
        this.sensor = new ColorSensorV3(port);

        this.counter = new Notifier(() -> {
            int currentColor = this.getCurrentColor();
            
            if (currentColor == this.lastColor)
                return;
            
            this.lastColor = currentColor;
            this.colorChanges++;
        });

        this.counter.startPeriodic(20f / 1000f);
    }

    public void startCountingColorChanges()
    {
        this.colorChanges = 0;
    }

    public int getColorChangeCount()
    {
        return this.colorChanges;
    }

    private int getMinValue(int[] arr)
    {
        int index = 0;
        int min = 0;
        for (int i = 0; i < arr.length; i++)
        {
            if (index == 0 || arr[i] < min)
            {
                index = i;
                min = arr[i];
            }
        }
        return index;
    }

    public int[] getRawData()
    {
        int[] RGB_IR_PROX = {
            this.sensor.getRed(), this.sensor.getGreen(), this.sensor.getBlue(),
            this.sensor.getIR(), this.sensor.getProximity()
        };
        return RGB_IR_PROX;
    }

    public int[] getRawColor()
    {
        int[] RGB = {this.sensor.getRed(), this.sensor.getGreen(), this.sensor.getBlue()};
        return RGB;
    }

    public int getCurrentColor()
    {
        // check if it even exists
        if (this.sensor.getProximity() > this.MAX_PROXIMITY)
            return this.NONE;

        // raw values read from sensor
        int[] RGB = this.getRawColor();

        if (RGB[0] == 0 && RGB[1] == 0 && RGB[2] == 0)
            return this.NONE;

        // how far are colors from what they should be?
        int[] RGBYScore = {
            (255 - RGB[0]) + (RGB[1] - 0  ) + (RGB[2] - 0  ), // perfect red    = (255, 0  , 0  )
            (RGB[0] - 0  ) + (255 - RGB[1]) + (RGB[2] - 0  ), // perfect green  = (0  , 255, 0  )
            (RGB[0] - 0  ) + (RGB[1] - 0  ) + (255 - RGB[2]), // perfect blue   = (0  , 0  , 255)
            (255 - RGB[0]) + (RGB[1] - 0  ) + (255 - RGB[2]), // perfect yellow = (255, 0  , 255)
        };

        // return the closest color
        return this.getMinValue(RGBYScore);
    }

    public String getColorName(int color)
    {
        /* Someone want to explain why this doesn't work?

        switch (color)
        {
            case this.NONE:
                return "NONE";
            case this.RED:
                return "RED";
            ...
            default:
                return "UNKNOWN";
        }
        */

        if (color == this.NONE)
            return "NONE";
        else if (color == this.RED)
            return "RED";
        else if (color == this.GREEN)
            return "GREEN";
        else if (color == this.BLUE)
            return "BLUE";
        else if (color == this.YELLOW)
            return "YELLOW";
        else
            return "UNKNOWN";
    }
}