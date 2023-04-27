package frc.robot;
import edu.wpi.first.wpilibj.XboxController;

public class DoubleController {
    XboxController c1;
    XboxController c2;

    public DoubleController(XboxController controller1, XboxController controller2){
        c1 = controller1;
        c2 = controller2;

    }
    //Code uses multiple functions to be simple when implemented in other files
    //Please don't change this into a single function with a parameter
    public double getLeftX(){
        return(Math.max(c1.getLeftX(), c2.getLeftX()));
    }
    public double getLeftY(){
        return(Math.max(c1.getLeftY(), c2.getLeftY()));
    }
    public double getRightX(){
        return(Math.max(c1.getRightX(), c2.getRightX()));
    }
    public double getRightY(){
        return(Math.max(c1.getRightY(), c2.getRightY()));
    }
    public double getLeftTrigger(){
        //The raw axis 2 represents the angle the trigger is at
        return(Math.max(c1.getRawAxis(2), c2.getRawAxis(2)));
    }
    public double getRightTrigger(){
        //The raw axis 3 represents the angle the trigger is at
        return(Math.max(c1.getRawAxis(3), c2.getRawAxis(3)));
    }
    public boolean getXButton(){
        boolean temp = (c1.getXButton()||c2.getXButton());
        return temp;
    }
    public boolean getAButton(){
        boolean temp = (c1.getAButton()||c2.getAButton());
        return temp;
    }
    public boolean getBButton(){
        boolean temp = (c1.getBButton()||c2.getBButton());
        return temp;
    }
    public boolean getYButton(){
        boolean temp = (c1.getYButton()||c2.getYButton());
        return temp;
    }
    public boolean getLeftBumper(){
        boolean temp = (c1.getLeftBumper()||c2.getLeftBumper());
        return temp;
    }
    public boolean getRightBumper(){
        boolean temp = (c1.getRightBumper()||c2.getRightBumper());
        return temp;
    }
    public boolean getViewButton(){
        //Small button under xbox logo button to the left
        boolean temp = (c1.getRawButton(7)||c2.getRawButton(8));
        return temp;
    }
    public boolean getMenuButton(){
        //Small button under xbox logo button to the right
        boolean temp = (c1.getXButton()||c2.getXButton());
        return temp;
    }
    public boolean getDUp(){
        boolean temp;
        if(c1.getPOV() == 0 || c2.getPOV() == 0){
            temp = true;
        }else{
            temp = false;
        }
        return temp;
    }
    public boolean getDRight(){
        boolean temp;
        if(c1.getPOV() == 90 || c2.getPOV() == 90){
            temp = true;
        }else{
            temp = false;
        }
        return temp;
    }
    public boolean getDDown(){
        boolean temp;
        if(c1.getPOV() == 180 || c2.getPOV() == 180){
            temp = true;
        }else{
            temp = false;
        }
        return temp;
    }
    public boolean getDLeft(){
        boolean temp;
        if(c1.getPOV() == 270 || c2.getPOV() == 270){
            temp = true;
        }else{
            temp = false;
        }
        return temp;
    }
    public boolean getDUpRight(){
        boolean temp;
        if(c1.getPOV() == 45 || c2.getPOV() == 45){
            temp = true;
        }else{
            temp = false;
        }
        return temp;
    }
    public boolean getDDownRight(){
        boolean temp;
        if(c1.getPOV() == 135 || c2.getPOV() == 135){
            temp = true;
        }else{
            temp = false;
        }
        return temp;
    }
    public boolean getDDownLeft(){
        boolean temp;
        if(c1.getPOV() == 225 || c2.getPOV() == 225){
            temp = true;
        }else{
            temp = false;
        }
        return temp;
    }
    public boolean getUpLeft(){
        boolean temp;
        if(c1.getPOV() == 315 || c2.getPOV() == 315){
            temp = true;
        }else{
            temp = false;
        }
        return temp;
    }
}
