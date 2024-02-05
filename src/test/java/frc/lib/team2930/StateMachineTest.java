package frc.lib.team2930;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.Test;

public class StateMachineTest {
  static class SM1 extends StateMachine {
    boolean gotState1;
    boolean gotState2;
    boolean gotState3;

    SM1() {
      setInitialState(this::state1);
    }

    StateHandler state1() {
      gotState1 = true;
      System.out.printf("SM1:state1: %s\n", timeFromStart());
      return this::state2;
    }

    StateHandler state2() {
      gotState2 = true;
      System.out.printf("SM1:state2: %s\n", timeFromStart());
      return suspendForSubStateMachine(new SM2(), this::state3);
    }

    StateHandler state3(StateMachine subStateMachine) {
      gotState3 = true;
      System.out.printf("SM1:state3: %s\n", timeFromStart());

      if (subStateMachine.wasStopped()) {
        return setDone();
      } else {
        return setStopped();
      }
    }
  }

  static class SM2 extends StateMachine {
    SM2() {
      setInitialState(this::state1);
    }

    StateHandler state1() {
      System.out.println("SM2:State1");
      return this::state2;
    }

    StateHandler state2() {
      System.out.println("SM2:State2");
      return setStopped();
    }
  }

  static class SM3 extends StateMachine {
    boolean gotState1;
    boolean gotState2;
    boolean gotEvent1;
    boolean gotEvent2;

    SM3() {
      setInitialState(this::state1);
    }

    StateHandler state1() {
      assertFalse(gotState2);

      gotState1 = true;
      Timer.delay(0.1);
      return null;
    }

    StateHandler state2() {
      assertTrue(gotState1);

      gotState2 = true;
      return setDone();
    }

    @TimerEvent(after = 0.2)
    StateHandler event1() {
      assertFalse(gotEvent2);

      gotEvent1 = true;
      System.out.printf("SM3:event1: %s\n", timeFromStart());
      return null;
    }

    @TimerEvent(after = 0.4)
    StateHandler event2() {
      gotEvent2 = true;
      System.out.printf("SM3:event2: %s\n", timeFromStart());
      return this::state2;
    }
  }

  static class SM4 extends StateMachine {
    boolean gotState1;
    boolean gotState2;
    boolean gotCmd;

    SM4() {
      setInitialState(this::state1);
    }

    StateHandler state1() {
      gotState1 = true;
      return suspendForCommand(
          new Command() {
            @Override
            public void execute() {
              gotCmd = true;
            }

            @Override
            public boolean isFinished() {
              return gotCmd;
            }

            public boolean runsWhenDisabled() {
              return true;
            }
          },
          this::state2);
    }

    StateHandler state2(Command command) {
      assertTrue(gotState1);

      gotState2 = true;
      return setDone();
    }
  }

  @Test()
  public void testStateMachine() {
    var sm = new SM2();
    while (sm.isRunning()) {
      sm.advance();
    }
  }

  @Test()
  public void testChainingStateMachine2() {
    var sm = new SM1();
    while (sm.isRunning()) {
      sm.advance();
    }

    assertTrue(sm.gotState1);
    assertTrue(sm.gotState2);
    assertTrue(sm.gotState3);
    assertFalse(sm.wasStopped());
  }

  @Test()
  public void testTimers() {
    var sm = new SM3();
    while (sm.isRunning()) {
      sm.advance();
    }

    assertTrue(sm.gotState1);
    assertTrue(sm.gotState2);
    assertTrue(sm.gotEvent1);
    assertTrue(sm.gotEvent2);
  }

  @Test()
  public void testSubCommands() throws InterruptedException {
    var scheduler = CommandScheduler.getInstance();
    var quit = new AtomicBoolean();

    var thread =
        new Thread(
            () -> {
              while (!quit.get()) {
                scheduler.run();
              }
            });

    thread.start();

    var sm = new SM4();
    while (sm.isRunning()) {
      sm.advance();
    }

    assertTrue(sm.gotState1);
    assertTrue(sm.gotState2);
    assertTrue(sm.gotCmd);

    quit.set(true);
    thread.join();
  }
}
