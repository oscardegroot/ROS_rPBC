

#pragma once

#include <string>
#include <franka/exception.h>

// Base exception
// struct Exception : public std::runtime_error {
//   using std::runtime_error::runtime_error;
// };

/* Exceptions used for added safety */
struct SafetyException : public franka::Exception{
	using franka::Exception::Exception;
};


struct BoundException : public SafetyException{
	using SafetyException::SafetyException;
};

struct VelocityElementBoundException : public SafetyException{
	using SafetyException::SafetyException;
};

struct EEVelocityBoundException : public SafetyException{
	using SafetyException::SafetyException;
};

struct TorqueBoundException : public SafetyException{
	using SafetyException::SafetyException;
};
