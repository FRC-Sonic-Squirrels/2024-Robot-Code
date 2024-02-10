package frc.lib.team2930;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.team2930.commands.WaitForRequirementsCommand;
import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class CommandTest
{
  public static class TestRequirement implements Subsystem
  {
  }

  public static class TestCommand extends Command
  {
    public int executed;
    public boolean done;

    @Override
    public boolean runsWhenDisabled()
    {
      return true;
    }

    @Override
    public void execute()
    {
      executed++;
    }

    @Override
    public boolean isFinished()
    {
      return done;
    }
  }

  @Test()
  public void testSubCommandsWithConflictingRequirements() throws InterruptedException {
    var scheduler = CommandScheduler.getInstance();

    var cmd1 = new TestCommand();
    var cmd2 = new TestCommand();
    var cmd3 = new TestCommand();

    var req1 = new TestRequirement();
    var req2 = new TestRequirement();
    var req3 = new TestRequirement();

    cmd1.addRequirements(req1);
    cmd2.addRequirements(req1, req2);
    cmd3.addRequirements(req2, req3);

    cmd1.schedule();
    scheduler.run();
    scheduler.run();
    assertTrue(cmd1.executed > 0);
    assertFalse(cmd2.executed > 0);
    assertFalse(cmd3.executed > 0);

    new WaitForRequirementsCommand(cmd2).schedule();
    scheduler.run();
    scheduler.run();
    assertTrue(cmd1.executed > 0);
    assertFalse(cmd2.executed > 0);
    assertFalse(cmd3.executed > 0);

    new WaitForRequirementsCommand(cmd3).schedule();
    scheduler.run();
    scheduler.run();
    assertTrue(cmd1.executed > 0);
    assertFalse(cmd2.executed > 0);
    assertTrue(cmd3.executed > 0);

    cmd1.done = true;
    scheduler.run();
    scheduler.run();
    assertFalse(cmd1.isScheduled());
    cmd3.done = true;
    scheduler.run();
    scheduler.run();
    scheduler.run();
    assertTrue(cmd2.executed > 0);
    assertTrue(cmd3.executed > 0);
  }
}
