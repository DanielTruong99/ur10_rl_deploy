#include "fsm.hpp"

namespace fsm
{
    void FSM::dispatch(BuiltInEvent event)
    {
        StateFunction prev_state = _state;
        Status status = (this->*_state)(event);

        if (status == Status::TRAN_STATUS)
        {
            (this->*prev_state)(BuiltInEvent::EXIT_SIG);
            (this->*_state)(BuiltInEvent::ENTRY_SIG);
        }
    }

    void FSM::transition_to(StateFunction new_state)
    {
        _state = new_state;
    }

    Status FSM::initial_state(BuiltInEvent event)
    {
        // Default implementation
        return Status::IGNORED_STATUS;
    }
} // namespace fsm
