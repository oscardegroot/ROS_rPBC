

#pragma once

#include <string>
#include <franka/exception.h>

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


/* Other Exceptions */
struct OperationalException : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

struct RetrievalException : public OperationalException {
  using OperationalException::OperationalException;
};

struct ParameterException : public OperationalException {
  using OperationalException::OperationalException;
};

struct RegisteringException : public OperationalException {
    using OperationalException::OperationalException;
};


