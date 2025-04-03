#ifndef FSM_HPP
#define FSM_HPP

#include <memory>

namespace fsm
{

    enum class BuiltInEvent
    {
        DEFAULT_SIG = 1,
        ENTRY_SIG = 2,
        EXIT_SIG = 3,
        USER_SIG = 4
    };

    enum class Status
    {
        TRAN_STATUS = 1,
        HANDLED_STATUS = 2,
        IGNORED_STATUS = 3,
        INIT_STATUS = 4
    };

    class FSM
    {
        public:
            using StateFunction = Status (FSM::*)(BuiltInEvent);

            FSM() : _state(&FSM::initial_state) {}

            void dispatch(BuiltInEvent event);

            void transition_to(StateFunction new_state);

        protected:
            virtual Status initial_state(BuiltInEvent event);

        private:
            StateFunction _state;
    };

}
#endif // FSM_HPP
