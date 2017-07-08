#include "State.h"
#include "StateVar.h"
#include <iostream>

void State::Initialize(int N)
{
	states_.clear();
	CreateAndAddNewState(x_, 0, N);
	CreateAndAddNewState(y_, N);
	CreateAndAddNewState(psi_, N);
	CreateAndAddNewState(v_, N);
	CreateAndAddNewState(cte_, N);
	CreateAndAddNewState(epsi_, N);
	CreateAndAddNewState(delta_, N, N - 1);
	CreateAndAddNewState(a_, N - 1, N - 1);
}

void State::CreateAndAddNewState(StateVar& state, int N, int max_states)
{
	auto newIndex = N == 0 ? 0 : (*states_.back()).start_index_ + N;
	state.max_ = max_states == 0 ? N : max_states;
	state.start_index_ = newIndex;
	states_.push_back(&state);
}

void State::CreateAndAddNewState(StateVar& state, int N)
{
	CreateAndAddNewState(state, N, 0);
}

State::State(int N)
{
	Initialize(N);
}

State::State()
{
}

void State::SetValues(const ADvector& vars, int offset)
{
	for (int i = 0, m = states_.size(); i <m; i++)
		SetStateValue(vars, offset, *states_[i]);
}

void State::SetStateValue(const ADvector& vars, int offset, StateVar& state)
{
	if (state.max_ > offset)
		state.val_ = vars[state.start_index_ + offset];
}
