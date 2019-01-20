package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class pneumaticSystem {
    // First solenoid will operate at port 0 and 2
    // Second solenoid will operate at port 1 and 3
    DoubleSolenoid firstSolenoid = new DoubleSolenoid(0, 2);
    DoubleSolenoid secondSolenoid = new DoubleSolenoid(1, 3);

    public void pistonController(boolean userCmd){
        if (userCmd == true){
            
        }
    }
}