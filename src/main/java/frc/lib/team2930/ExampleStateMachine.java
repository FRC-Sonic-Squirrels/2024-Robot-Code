package frc.lib.team2930;

public class ExampleStateMachine
{
    static class SM1 extends StateMachine
    {
        SM1()
        {
            setInitialState(this::state1);
        }

        StateHandler state1()
        {
            System.out.println("State1");
            return this::state2;
        }

        StateHandler state2()
        {
            System.out.println("State2");
            return suspendForSubStateMachine(new SM2(), this::state3);
        }

        StateHandler state3(StateMachine subStateMachine)
        {
            System.out.println("State3");
            return setDone();
        }
    }

    static class SM2 extends StateMachine
    {
        SM2()
        {
            setInitialState(this::state1);
        }

        StateHandler state1()
        {
            System.out.println("SM2:State1");
            return this::state2;
        }

        StateHandler state2()
        {
            System.out.println("SM2:State2");
            return setStopped();
        }
    }

    public static void main(String[] args)
    {
        var sm = new SM1();
        while (sm.isRunning())
        {
            sm.advance();
        }
    }
}
