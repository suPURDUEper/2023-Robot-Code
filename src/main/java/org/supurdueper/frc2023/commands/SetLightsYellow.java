package org.supurdueper.frc2023.commands;
import org.supurdueper.frc2023.subsystems.Lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
public class SetLightsYellow extends CommandBase{
    public final Lights lights;

public SetLightsYellow(Lights lights) {
    this.lights = lights;
    addRequirements(lights);
}
@Override
public void initialize(){
    lights.setLightsYellow();
}
}

