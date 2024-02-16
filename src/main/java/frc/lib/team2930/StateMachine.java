package frc.lib.team2930;

import static java.lang.annotation.ElementType.METHOD;
import static java.lang.annotation.RetentionPolicy.RUNTIME;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.lang.annotation.Retention;
import java.lang.annotation.Target;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Comparator;
import org.littletonrobotics.junction.Logger;

public class StateMachine {
  public enum Status {
    Running,
    Done,
    Stopped
  }

  @Retention(RUNTIME)
  @Target(METHOD)
  public @interface TimerEvent {
    double after();
  }

  @FunctionalInterface
  public interface StateHandler {
    StateHandler advance();
  }

  @FunctionalInterface
  public interface ResumeStateHandler {
    StateHandler advance(StateMachine subStateMachine);
  }

  @FunctionalInterface
  public interface ResumeStateHandlerFromCommand {
    StateHandler advance(Command command);
  }

  static class EventState {
    EventState next;
    Method md;
    double trigger;

    EventState(Method md, TimerEvent anno) {
      this.md = md;
      this.trigger = anno.after();
    }
  }

  private Status status;
  private StateHandler currentState;
  private double startTime;
  private EventState nextEvent;

  protected StateMachine() {
    var events = new ArrayList<EventState>();

    for (Method md : this.getClass().getDeclaredMethods()) {
      var anno = md.getAnnotation(TimerEvent.class);
      if (anno != null) {
        events.add(new EventState(md, anno));
      }
    }

    // Sort in increasing time.
    events.sort(Comparator.comparing((v) -> v.trigger));

    // Chain them in time order, to make it easier to process them.
    EventState previous = null;

    for (var event : events) {
      if (previous == null) {
        nextEvent = event;
      } else {
        previous.next = event;
      }

      previous = event;
    }
  }

  protected void setInitialState(StateHandler initialState) {
    this.currentState = initialState;
    this.status = Status.Running;
    this.startTime = Timer.getFPGATimestamp();
  }

  public Command asCommand() {
    return new Command() {
      @Override
      public void initialize() {
        StateMachine.this.startTime = Timer.getFPGATimestamp();
      }

      @Override
      public void execute() {
        advance();
      }

      public boolean isFinished() {
        return !isRunning();
      }
    };
  }

  public void advance() {
    while (isRunning()) {
      processEvents();

      var nextState = currentState.advance();
      if (nextState == null) break;

      currentState = nextState;
    }
  }

  private void processEvents() {
    var elapsedTime = timeFromStart();

    for (var event = nextEvent; event != null; event = event.next) {
      if (event.trigger > elapsedTime) {
        nextEvent = event;
        break;
      }

      try {
        var nextState = (StateHandler) event.md.invoke(this);
        if (nextState != null) {
          this.currentState = nextState;
        }
      } catch (Throwable e) {
        e.printStackTrace();
      }
    }
  }

  public double timeFromStart() {
    return Timer.getFPGATimestamp() - startTime;
  }

  public boolean isRunning() {
    return status == Status.Running;
  }

  public boolean wasStopped() {
    return status == Status.Stopped;
  }

  protected StateHandler setDone() {
    status = Status.Done;
    return null;
  }

  protected StateHandler setStopped() {
    status = Status.Stopped;
    return null;
  }

  protected StateHandler suspendForSubStateMachine(
      StateMachine subStateMachine, ResumeStateHandler handler) {
    return () -> {
      subStateMachine.advance();
      Logger.recordOutput("Autonomous/subStateStatus", subStateMachine.status);
      if (subStateMachine.isRunning()) return null;

      return handler.advance(subStateMachine);
    };
  }

  protected StateHandler suspendForCommand(Command command, ResumeStateHandlerFromCommand handler) {
    command.schedule();

    return () -> {
      if (!command.isFinished()) return null;

      return handler.advance(command);
    };
  }

  protected void spawnCommand(Command command, ResumeStateHandlerFromCommand handler) {
    var sequence =
        command.andThen(
            () -> {
              if (!isRunning()) return;

              var nextState = handler.advance(command);
              if (nextState != null) {
                this.currentState = nextState;
              }
            });

    sequence.schedule();
  }
}
