package frc.lib.team2930;

public class StateMachine
{
    public enum Status
    {
        Running,
        Done,
        Stopped
    }

    @FunctionalInterface
    public interface StateHandler
    {
        StateHandler advance();
    }

    @FunctionalInterface
    public interface ResumeStateHandler
    {
        StateHandler advance(StateMachine subStateMachine);
    }

    private Status       status;
    private StateHandler currentState;

    protected void setInitialState(StateHandler initialState)
    {
        this.currentState = initialState;
        this.status = Status.Running;
    }

    public void advance()
    {
        while (isRunning())
        {
            var nextState = currentState.advance();
            if (nextState == null) break;

            currentState = nextState;
        }
    }

    public boolean isRunning()
    {
        return status == Status.Running;
    }

    public boolean wasStopped()
    {
        return status == Status.Stopped;
    }

    protected StateHandler setDone()
    {
        status = Status.Done;
        return null;
    }

    protected StateHandler setStopped()
    {
        status = Status.Stopped;
        return null;
    }

    protected StateHandler suspendForSubStateMachine(StateMachine subStateMachine,
                                                     ResumeStateHandler handler)
    {
        return () ->
        {
            subStateMachine.advance();
            if (subStateMachine.isRunning()) return null;

            return handler.advance(subStateMachine);
        };
    }
}
