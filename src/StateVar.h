#pragma once
#include <cppad/cppad.hpp>

class StateVar
{
public:

	StateVar();
	virtual ~StateVar();

	int start_index_;
	int max_;
	CppAD::AD <double> val_;
};