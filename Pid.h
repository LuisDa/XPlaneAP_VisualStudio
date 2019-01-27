#pragma once

class PID
{
public:
	PID(float prop_gain, float int_gain, float der_gain);
	void setPropGain(float prop_gain);
	void setDerGain(float der_gain);
	float getOutput(float error, float delta_time);

private:
	float m_propGain;
	float m_derGain;
	float m_prevErr; //Para la componente derivativa
};

PID::PID(float prop_gain, float int_gain, float der_gain)
{
	m_propGain = prop_gain;

}

void PID::setPropGain(float prop_gain)
{
	m_propGain = prop_gain;
}

void PID::setDerGain(float der_gain)
{
	m_derGain = der_gain;
}

float PID::getOutput(float error, float delta_time)
{
	float output = m_propGain * error  + m_derGain * (error - m_prevErr)/delta_time;
	m_prevErr = error;
	return output;
}