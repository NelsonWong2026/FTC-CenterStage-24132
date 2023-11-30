package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Claw;

public class MoveUp extends CommandBase {

    private final Claw m_claw;

    public MoveUp(Claw claw) {
        m_claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        m_claw.moveUp();
    }

    @Override
    public boolean isFinished() {
        return true;
    }


}
