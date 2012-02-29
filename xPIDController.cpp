/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

// sample robot code
// Steve Tarr - team 1425 mentor - 29-Jan-2012
//
// xPIDController is a copy of the WPILib PIDController class with some extra
// debugging interfaces.

#include "xPIDController.h"
#include "NetworkCommunication/UsageReporting.h"
#include "Notifier.h"
#include "PIDSource.h"
#include "PIDOutput.h"
#include <math.h>
#include "Synchronized.h"
/**
 * Allocate a PID object with the given constants for P, I, D
 * @param Kp the proportional coefficient
 * @param Ki the integral coefficient
 * @param Kd the derivative coefficient
 * @param source The PIDSource object that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly effects calculations of the
 * integral and differental terms. The default is 50ms.
 */
xPIDController::xPIDController(float Kp, float Ki, float Kd,
								PIDSource *source, PIDOutput *output,
								float period) :
	m_semaphore (0)
{
	m_semaphore = semBCreate(SEM_Q_PRIORITY, SEM_FULL);

	m_controlLoop = new Notifier(xPIDController::CallCalculate, this);

	m_P = Kp;
	m_I = Ki;
	m_D = Kd;
	m_maximumOutput = 1.0;
	m_minimumOutput = -1.0;

	m_maximumInput = 0;
	m_minimumInput = 0;

	m_continuous = false;
	m_enabled = false;
	m_setpoint = 0;

	m_prevError = 0;
	m_totalError = 0;
	m_tolerance = .05;

	m_result = 0;

	m_pidInput = source;
	m_pidOutput = output;
	m_period = period;

	m_controlLoop->StartPeriodic(m_period);

	static INT32 instances = 0;
	instances++;
	nUsageReporting::report(nUsageReporting::kResourceType_PIDController, instances);
}

/**
 * Free the PID object
 */
xPIDController::~xPIDController()
{
	semFlush(m_semaphore);
	delete m_controlLoop;
}

/**
 * Call the Calculate method as a non-static method. This avoids having to prepend
 * all local variables in that method with the class pointer. This way the "this"
 * pointer will be set up and class variables can be called more easily.
 * This method is static and called by the Notifier class.
 * @param controller the address of the PID controller object to use in the background loop
 */
void xPIDController::CallCalculate(void *controller)
{
	xPIDController *control = (xPIDController*) controller;
	control->Calculate();
}

 /**
  * Read the input, calculate the output accordingly, and write to the output.
  * This should only be called by the Notifier indirectly through CallCalculate
  * and is created during initialization.
  */	
void xPIDController::Calculate()
{
	bool enabled;
	PIDSource *pidInput;

	CRITICAL_REGION(m_semaphore)
	{
		if (m_pidInput == 0) return;
		if (m_pidOutput == 0) return;
		enabled = m_enabled;
		pidInput = m_pidInput;
	}
	END_REGION;

	if (enabled)
	{
		float input = pidInput->PIDGet();
		float result;
		PIDOutput *pidOutput;

		{
			Synchronized sync(m_semaphore);
			m_input = input;
			m_error = m_setpoint - input;
			if (m_continuous)
			{
				if (fabs(m_error) > (m_maximumInput - m_minimumInput) / 2)
				{
					if (m_error > 0)
					{
						m_error = m_error - m_maximumInput + m_minimumInput;
					}
					else
					{
						m_error = m_error + m_maximumInput - m_minimumInput;
					}
				}
			}

			double potentialIGain = (m_totalError + m_error) * m_I;
			if (potentialIGain < m_maximumOutput)
			{
				if (potentialIGain > m_minimumOutput)
					m_totalError += m_error;
				else
					m_totalError = m_minimumOutput / m_I;
			}
			else
			{
				m_totalError = m_maximumOutput / m_I;
			}

			m_result = m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError);
			m_prevError = m_error;

			if (m_result > m_maximumOutput) m_result = m_maximumOutput;
			else if (m_result < m_minimumOutput) m_result = m_minimumOutput;

			pidOutput = m_pidOutput;
			result = m_result;
		}

		pidOutput->PIDWrite(result);
	}
}

/**
 * Set the PID Controller gain parameters.
 * Set the proportional, integral, and differential coefficients.
 * @param p Proportional coefficient
 * @param i Integral coefficient
 * @param d Differential coefficient
 */
void xPIDController::SetPID(float p, float i, float d)
{
	CRITICAL_REGION(m_semaphore)
	{
		m_P = p;
		m_I = i;
		m_D = d;
	}
	END_REGION;
}

/**
 * Get the Proportional coefficient
 * @return proportional coefficient
 */
float xPIDController::GetP()
{
	CRITICAL_REGION(m_semaphore)
	{
		return m_P;
	}
	END_REGION;
}

/**
 * Get the Integral coefficient
 * @return integral coefficient
 */
float xPIDController::GetI()
{
	CRITICAL_REGION(m_semaphore)
	{
		return m_I;
	}
	END_REGION;
}

/**
 * Get the Differential coefficient
 * @return differential coefficient
 */
float xPIDController::GetD()
{
	CRITICAL_REGION(m_semaphore)
	{
		return m_D;
	}
	END_REGION;
}

/**
 * Return the current PID result
 * This is always centered on zero and constrained the the max and min outs
 * @return the latest calculated output
 */
float xPIDController::Get()
{
	float result;
	CRITICAL_REGION(m_semaphore)
	{
		result = m_result;
	}
	END_REGION;
	return result;
}

/**
 *  Set the PID controller to consider the input to be continuous,
 *  Rather then using the max and min in as constraints, it considers them to
 *  be the same point and automatically calculates the shortest route to
 *  the setpoint.
 * @param continuous Set to true turns on continuous, false turns off continuous
 */
void xPIDController::SetContinuous(bool continuous)
{
	CRITICAL_REGION(m_semaphore)
	{
		m_continuous = continuous;
	}
	END_REGION;

}

/**
 * Sets the maximum and minimum values expected from the input.
 * 
 * @param minimumInput the minimum value expected from the input
 * @param maximumInput the maximum value expected from the output
 */
void xPIDController::SetInputRange(float minimumInput, float maximumInput)
{
	CRITICAL_REGION(m_semaphore)
	{
		m_minimumInput = minimumInput;
		m_maximumInput = maximumInput;	
	}
	END_REGION;

	SetSetpoint(m_setpoint);
}

/**
 * Sets the minimum and maximum values to write.
 * 
 * @param minimumOutput the minimum value to write to the output
 * @param maximumOutput the maximum value to write to the output
 */
void xPIDController::SetOutputRange(float minimumOutput, float maximumOutput)
{
	CRITICAL_REGION(m_semaphore)
	{
		m_minimumOutput = minimumOutput;
		m_maximumOutput = maximumOutput;
	}
	END_REGION;
}

/**
 * Set the setpoint for the PIDController
 * @param setpoint the desired setpoint
 */
void xPIDController::SetSetpoint(float setpoint)
{
	CRITICAL_REGION(m_semaphore)
	{
		if (m_maximumInput > m_minimumInput)
		{
			if (setpoint > m_maximumInput)
				m_setpoint = m_maximumInput;
			else if (setpoint < m_minimumInput)
				m_setpoint = m_minimumInput;
			else
				m_setpoint = setpoint;
		}
		else
		{
			m_setpoint = setpoint;
		}
	}
	END_REGION;	
}

/**
 * Returns the current setpoint of the PIDController
 * @return the current setpoint
 */
float xPIDController::GetSetpoint()
{
	float setpoint;
	CRITICAL_REGION(m_semaphore)
	{
		setpoint = m_setpoint;
	}
	END_REGION;
	return setpoint;
}

/**
 * Returns the current input
 * @return the current input
 */
float xPIDController::GetInput()
{
	float input;
	CRITICAL_REGION(m_semaphore)
	{
		input = m_input;
	}
	END_REGION;
	return input;
}

/**
 * Returns the current difference of the input from the setpoint
 * @return the current error
 */
float xPIDController::GetError()
{
	float error;
	CRITICAL_REGION(m_semaphore)
	{
		error = m_error;
	}
	END_REGION;
	return error;
}

/*
 * Set the percentage error which is considered tolerable for use with
 * OnTarget.
 * @param percentage error which is tolerable
 */
void xPIDController::SetTolerance(float percent)
{
	CRITICAL_REGION(m_semaphore)
	{
		m_tolerance = percent;
	}
	END_REGION;
}

/*
 * Return true if the error is within the percentage of the total input range,
 * determined by SetTolerance. This asssumes that the maximum and minimum input
 * were set using SetInput.
 */
bool xPIDController::OnTarget()
{
	bool temp;
	CRITICAL_REGION(m_semaphore)
	{
		temp = fabs(m_error) < (m_tolerance / 100 * 
			(m_maximumInput - m_minimumInput));
	}
	END_REGION;
	return temp;
}

/**
 * Begin running the PIDController
 */
void xPIDController::Enable()
{
	CRITICAL_REGION(m_semaphore)
	{			
		m_enabled = true;
	}
	END_REGION;	
}
/**
 * Stop running the PIDController, this sets the output to zero before stopping.
 */
void xPIDController::Disable()
{
	CRITICAL_REGION(m_semaphore)
	{
		m_pidOutput->PIDWrite(0);
		m_enabled = false;
	}
	END_REGION;
}

/**
 * Return true if PIDController is enabled.
 */
bool xPIDController::IsEnabled()
{
	bool enabled;
	CRITICAL_REGION(m_semaphore)
	{
		enabled = m_enabled;
	}
	END_REGION;
	return enabled;
}

/**
 * Reset the previous error,, the integral term, and disable the controller.
 */
void xPIDController::Reset()
{
	Disable();

	CRITICAL_REGION(m_semaphore)
	{
		m_prevError = 0;
		m_totalError = 0;
		m_result = 0;
	}
	END_REGION;
}
