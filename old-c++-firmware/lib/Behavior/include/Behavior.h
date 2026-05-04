#pragma once

#include <memory>

#include "Hal.h"

template <typename State>
class StateMachine;


/// Base for all 
template <typename State>
class StateBase {
protected:
    /// Marker class that the react methods return
    class StateTransition {
    public:
        static constexpr StateTransition no_change = StateTransition(false, nullptr);
        static constexpr StateTransition terminate = StateTransition(true, nullptr);
        template<typename NewStateT, typename... Args>
        static StateTransition new_state(Args&&... args) {
            return StateTransition(true, std::make_unique<NewStateT>(std::forward<Args>(args)...));
        }

        bool is_change() const { return change; }

        std::unique_ptr<StateBase> take_state() {
            return std::move(newState);
        }

    private:
        constexpr StateTransition(bool change, std::unique_ptr<State> newState)
        : change(change), newState(std::move(newState))
        {}

        bool change;
        std::unique_ptr<State> newState;

        friend class StateMachine<State>;
    };

    friend class StateMachine<State>;
};

template <typename State>
class StateMachine {
public:
    using Clock = Hal::Clock;

    template <typename Event>
    void react(const Event& e) {
        auto transition = state->send_event(e);
        if (transition.is_change())
            state = transition.take_state();
    }

private:
    std::unique_ptr<State> state;
};
