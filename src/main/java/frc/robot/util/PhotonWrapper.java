package frc.robot.util;

import java.sql.Driver;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.DriverStation;

public class PhotonWrapper extends PhotonCamera 
{
    public PhotonWrapper(String name) 
    {
        super(name);
    }

    public void setPipline (){
        if(DriverStation.getAlliance().toString().equals("Red"))
        {
            this.setPipelineIndex(1);
        }
        else
        {
            this.setPipelineIndex(2);
        }
        
    }
    
   

    
}
