// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Turret.TurretState;

public class LEDs extends SubsystemBase {

  /** Creates a new LEDSubsystem. */

  public AddressableLED led;
  public AddressableLEDBuffer lBuffer;

  public final int LED_PORT = 1;
  public final int LED_LENGTH = 56;
  public final int TURRET_LED_LENGTH = 39;
  public int counter = 0;

  public final Color kRed = RBGToColor(new int[]{255, 0, 0});
  public final Color kBlue = RBGToColor(new int[]{0, 0, 255});
  public final Color kGreen = RBGToColor(new int[]{0, 255, 0});
  public final Color kBlack = RBGToColor(new int[]{0, 0, 0});
  public final Color kMVRTPurple = RBGToColor(new int[]{160, 0, 196});
  public final Color kMVRTGold = RBGToColor(new int[]{100, 35, 0});


  public enum LedState {CLIMBER, SHOOTING, DEFAULT};
  public LedState prevState = LedState.DEFAULT;
  public LedState currState = LedState.DEFAULT;

  public LEDs() 
  {
    led = new AddressableLED(LED_PORT);
    lBuffer = new AddressableLEDBuffer(LED_LENGTH + TURRET_LED_LENGTH);

    led.setLength(LED_LENGTH + TURRET_LED_LENGTH);
    led.start();

    setGradient(LED_LENGTH, LED_LENGTH + (TURRET_LED_LENGTH/3) - 1, kBlack, kGreen);
    setGradient(LED_LENGTH + (TURRET_LED_LENGTH/3), LED_LENGTH + ((TURRET_LED_LENGTH*2)/3) - 1, kBlack, kGreen);
    setGradient(LED_LENGTH + ((TURRET_LED_LENGTH*2)/3), LED_LENGTH + TURRET_LED_LENGTH - 1, kBlack, kGreen);
  }

  public Color RBGToColor(int[] values) 
  {
    return new Color(values[0]/255.0, values[1]/255.0, values[2]/255.0);
  }

  public int[] ColorToRGB(Color color) 
  {
    return new int[]{(int)(color.red * 255.0), (int)(color.green * 255.0), (int)(color.blue * 255.0)};
  }

  public void setLEDBulb(int index, Color color) 
  {
    lBuffer.setLED(index, color);
  }

  public Color getLEDColor(int index)
  {
    return lBuffer.getLED(index);
  }

  public void sendData() 
  {
    led.setData(lBuffer);
  }

  public void setSingleBlock(int startIndex, int endIndex, Color color) 
  {
    if(startIndex >= 0 && endIndex < LED_LENGTH) {
      for(int i = startIndex; i <= endIndex; i++)
      {
        lBuffer.setLED(i, color);
      }
    }
  }

  public void setFullLength(Color color)
  {
    setSingleBlock(0, LED_LENGTH-1, color);
  }

  public void moveUp(int startIndex, int endIndex, double delay)
  {
    Color tempColor = lBuffer.getLED(endIndex);
    
    for(int i = endIndex; i > startIndex; i--)
    {
      setLEDBulb(i, lBuffer.getLED(i-1));
    }

    setLEDBulb(startIndex, tempColor);

    sendData();
    Timer.delay(delay);
  }

  public void moveDown(int startIndex, int endIndex, double delay)
  {
    Color tempColor = lBuffer.getLED(startIndex);
    
    for(int i = startIndex; i < endIndex; i++)
    {
      setLEDBulb(i, lBuffer.getLED(i+1));
    }

    setLEDBulb(endIndex, tempColor);

    sendData();
    Timer.delay(delay);
  }

  public void setWave(int startIndex, int endIndex, int waveLength, Color darkColor, Color lightColor)
  {
    for(int i = startIndex; i < endIndex; i += waveLength)
    {
      setGradientOnTwoSides(i, i + waveLength - 1, darkColor, lightColor);
    }
  }

  public void setGradient(int startIndex, int endIndex, Color startColor, Color endColor)
  {
    int length = endIndex - startIndex;

    double redShift = (double)(endColor.red-startColor.red)/length;
    double greenShift = (double)(endColor.green-startColor.green)/length;
    double blueShift = (double)(endColor.blue-startColor.blue)/length;

    for(int i = 0; i <= length; i++)
    {
      setLEDBulb(i + startIndex, new Color((startColor.red+redShift*i), (startColor.green+greenShift*i), 
        (startColor.blue+blueShift*i)));
    }
  }

  public void setGradientOnTwoSides(int startIndex, int endIndex, Color endColor, Color centerColor)
  {
    int center = (endIndex + startIndex)/2;

    setGradient(center, endIndex, centerColor, endColor);
    setGradient(startIndex, center, endColor, centerColor);
  }

  public void setRainbow(int startIndex, int endIndex) 
  {
    int blockLength = (endIndex-startIndex)/3;
    setGradient(startIndex, startIndex + blockLength, kRed, kGreen);
    setGradient(startIndex + blockLength, endIndex - blockLength, kGreen, kBlue);
    setGradient(endIndex - blockLength, endIndex, kBlue, kRed);
  }

  public void setMultiBlock(int length, Color[] colors, int startIndex, int endIndex)
  {
    for(int i=startIndex; i <= endIndex; i += length)
    {
      setSingleBlock(i, i + length - 1, colors[(i/length) % colors.length]);
    }
  }

  public LedState getPreviousState() {
    return prevState;
  }

  public LedState getCurrentState() {
    return currState;
  }

  public void setPreviousState(LedState newState) {
    prevState = newState;
  }

  public void setCurrentState(LedState newState) {
    prevState = currState;
    currState = newState;
  }
  
  public void addGradientToColor(int index, double redAdd, double greenAdd, double blueAdd)
  {
    Color addColor = RBGToColor(new int[]{(int)redAdd, (int)greenAdd, (int)blueAdd});
    Color originalColor = getLEDColor(index);
    setLEDBulb(index, new Color((originalColor.red + addColor.red)%256, (originalColor.green + addColor.green)%256, 
      (originalColor.blue + addColor.blue)%256));
  }

  public Color getColorAtGradient(Color startColor, Color endColor, int length, int index) 
  {
    double redShift = (double)(endColor.red-startColor.red)/length;
    double greenShift = (double)(endColor.green-startColor.green)/length;
    double blueShift = (double)(endColor.blue-startColor.blue)/length;

    return new Color((startColor.red+redShift*index), (startColor.green+greenShift*index), 
        (startColor.blue+blueShift*index));
  }

  public void setFullLengthTurret(Color color)
  {
    setSingleBlock(LED_LENGTH, LED_LENGTH + TURRET_LED_LENGTH - 1, color);
  }

  public void setTurretLEDs(double flywheelSpeed, double targetSpeed, TurretState turretState)
  {
    if(turretState == TurretState.DISABLED)
      setFullLengthTurret(kRed);
    else if(flywheelSpeed == 0)
      setFullLengthTurret(kMVRTPurple);
    else if(turretState == TurretState.CAN_SHOOT || turretState == TurretState.TARGETING)
    {
      Color newColor = getColorAtGradient(kRed, kGreen, (int)targetSpeed, (int)flywheelSpeed);
      Color boldColor = kBlack;

      for(int i = LED_LENGTH; i < LED_LENGTH + TURRET_LED_LENGTH; i++)
      {
        if(getLEDColor(i).red >= boldColor.red && getLEDColor(i).green >= boldColor.green 
          && getLEDColor(i).blue >= boldColor.blue)
            boldColor = getLEDColor(i);
      }

      System.out.println(ColorToRGB(boldColor)[0] + " " + ColorToRGB(boldColor)[1] + " " + ColorToRGB(boldColor)[2]);
      System.out.println(ColorToRGB(newColor)[0] + " " + ColorToRGB(newColor)[1] + " " + ColorToRGB(newColor)[2]);

      for(int i = LED_LENGTH; i < LED_LENGTH + TURRET_LED_LENGTH; i++) {
        addGradientToColor(i, (newColor.red-boldColor.red), (newColor.green-boldColor.green), 
          (newColor.blue-boldColor.blue));
      }

      sendData();

      if(turretState == TurretState.CAN_SHOOT)
      {
        Color[] gradientColors = new Color[TURRET_LED_LENGTH];
        for(int i = LED_LENGTH; i < LED_LENGTH + TURRET_LED_LENGTH; i++)
        {
          gradientColors[i] = getLEDColor(i);
        }
          
        setFullLengthTurret(kBlack);
        sendData();

        Timer.delay(0.05);

        for(int i = LED_LENGTH; i < LED_LENGTH + TURRET_LED_LENGTH; i++)
        {
          setLEDBulb(i, gradientColors[i]);
        }
      }
    }

    moveUp(LED_LENGTH, LED_LENGTH + TURRET_LED_LENGTH - 1, 0.05);
    sendData(); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
