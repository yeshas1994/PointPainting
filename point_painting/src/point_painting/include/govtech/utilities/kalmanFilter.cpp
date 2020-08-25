#include "kalmanFilter.h"

namespace DosUtils
{

KalmanScalarFilter::KalmanScalarFilter(float startVal, float errVariance, float noiseVariance, float measureError)
: m_curVal(startVal), m_errVariance(errVariance), m_noiseVariance(noiseVariance), m_measureError(measureError)
{
}

float KalmanScalarFilter::run(float measureVal)
{
    float estimateVariance = m_errVariance + m_noiseVariance;
    float measureVariance = m_measureError * m_measureError;
    float kGain = estimateVariance / (estimateVariance + measureVariance);

    m_curVal = m_curVal + kGain * (measureVal - m_curVal);
    m_errVariance = (1.0f - kGain) * estimateVariance;

    return m_curVal;
}

float KalmanScalarFilter::getCurVal() const
{
    return m_curVal;
}

} // end of namespace DosUtils