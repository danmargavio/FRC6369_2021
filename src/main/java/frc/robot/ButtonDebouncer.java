package frc.robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;

public class ButtonDebouncer{

    Joystick joystick;
    int buttonnum;
    double latest;
    double debounce_period;
    boolean button = false;

    public ButtonDebouncer(Joystick joystick, int buttonnum){
        this.joystick = joystick;
        this.buttonnum = buttonnum;
        this.latest = 0;
        this.debounce_period = .5;
    }
    public ButtonDebouncer(Joystick joystick, int buttonnum, float period){
        this.joystick = joystick;
        this.buttonnum = buttonnum;
        this.latest = 0;
        this.debounce_period = period;
    }

    public void setDebouncePeriod(float period){
        this.debounce_period = period;
    }

    public boolean get(){
        double now = Timer.getFPGATimestamp();
        if(joystick.getRawButton(buttonnum)){
            if((now-latest) > debounce_period){
                latest = now;
                button = !button;
            }
        }
        return button;
    }
}