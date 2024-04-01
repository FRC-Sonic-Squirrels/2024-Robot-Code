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
import java.util.concurrent.atomic.AtomicBoolean;

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

  public interface StateHandlerWithName extends StateHandler {
    String getName();
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

  private static final LoggerGroup logGroup = LoggerGroup.build("StateMachine");

  private final String name;
  private final LoggerEntry.Text logName;
  private Status status;
  private StateHandler currentState;
  private double startTime;
  private double startTimeOfState;
  private EventState nextEvent;

  protected StateMachine(String name) {
    this.name = name;
    logName = logGroup.buildString(name);

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
    status = Status.Running;
    startTime = Timer.getFPGATimestamp();
    setNextState(initialState);
  }

  protected StateHandlerWithName stateWithName(String name, StateHandler handler) {
    return new StateHandlerWithName() {
      @Override
      public String getName() {
        return name;
      }

      @Override
      public StateHandler advance() {
        return handler.advance();
      }
    };
  }

  private void setNextState(StateHandler nextState) {
    currentState = nextState;
    startTimeOfState = Timer.getFPGATimestamp();

    if (currentState instanceof StateHandlerWithName stateWithName) {
      logName.info(stateWithName.getName());
    } else {
      logName.info(nextState.getClass().getName());
    }
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

      setNextState(nextState);
    }
  }

  public void logCurrentState(String name) {
    logName.info(currentState.getClass().getName());
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
          setNextState(nextState);
        }
      } catch (Throwable e) {
        e.printStackTrace();
      }
    }
  }

  public double timeFromStart() {
    return Timer.getFPGATimestamp() - startTime;
  }

  public double timeFromStartOfState() {
    return Timer.getFPGATimestamp() - startTimeOfState;
  }

  public boolean stateRunningLongerThan(double time) {
    return timeFromStartOfState() > time;
  }

  public boolean isRunning() {
    return status == Status.Running;
  }

  public boolean wasStopped() {
    return status == Status.Stopped;
  }

  protected StateHandler setDone() {
    status = Status.Done;
    logName.info("<done>");
    return null;
  }

  protected StateHandler setStopped() {
    status = Status.Stopped;
    logName.info("<stopped>");
    return null;
  }

  protected StateHandler suspendForSubStateMachine(
      StateMachine subStateMachine, ResumeStateHandler handler) {
    return stateWithName(
        "waitFor" + subStateMachine.name,
        () -> {
          subStateMachine.advance();
          if (subStateMachine.isRunning()) return null;

          return handler.advance(subStateMachine);
        });
  }

  protected StateHandler suspendForCommand(Command command, ResumeStateHandlerFromCommand handler) {
    AtomicBoolean done = new AtomicBoolean();

    spawnCommand(
        command,
        (c) -> {
          done.set(true);
          return null;
        });

    var name = command.getName();
    return stateWithName(
        "waitForCmd " + name,
        () -> {
          if (!done.get()) return null;

          return handler.advance(command);
        });
  }

  protected void spawnCommand(Command command, ResumeStateHandlerFromCommand handler) {
    var sequence =
        command.andThen(
            () -> {
              if (!isRunning()) return;

              var nextState = handler.advance(command);
              if (nextState != null) {
                setNextState(nextState);
              }
            });

    sequence.setName("substate_" + command.getName());
    sequence.schedule();
  }
}
