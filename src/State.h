#pragma once
#include <vector>
#include "StateVar.h"
#include <cppad/cppad.hpp>

using CppAD::AD;

class State
{

	void CreateAndAddNewState(StateVar& state, int N, int max_states);
	void CreateAndAddNewState(StateVar& state, int N);

public:

	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	explicit State(int N);
	State();
	void Initialize(int N);
	void SetValues(const ADvector& vars, int offset);
	void SetStateValue(const ADvector& vars, int offset, StateVar& state);

	std::vector<StateVar*> states_;

	// States of the kinetic motion model.
	StateVar x_;
	StateVar y_;
	StateVar psi_;
	StateVar v_;
	StateVar cte_;
	StateVar epsi_;
	StateVar delta_;
	StateVar a_;

};