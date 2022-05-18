#include "LambdaCamera.h"
#include "LambdaSyncCtrlObj.h"

using namespace lima;
using namespace lima::Lambda;

/*******************************************************************
 * \brief SyncCtrlObj constructor
 *******************************************************************/
SyncCtrlObj::SyncCtrlObj(Camera& cam) : 
		HwSyncCtrlObj(), 
		m_cam(cam)
{
	DEB_CONSTRUCTOR();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
SyncCtrlObj::~SyncCtrlObj()
{
}

//-----------------------------------------------------
//
//-----------------------------------------------------
bool SyncCtrlObj::checkTrigMode(TrigMode trig_mode)
{
	bool valid_mode = false;
	switch (trig_mode)
	{
		case IntTrig:
		case ExtTrigSingle:
		case ExtTrigMult:
		case ExtGate:
			valid_mode = true;
		break;

		default:
			valid_mode = false;
	}
	return valid_mode;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::setTrigMode(TrigMode trig_mode)
{
	DEB_MEMBER_FUNCT();
    if (!checkTrigMode(trig_mode))
        THROW_HW_ERROR(InvalidValue) << "Invalid " << DEB_VAR1(trig_mode);

	m_cam.setTrigMode(trig_mode);

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::getTrigMode(TrigMode& trig_mode)
{
	m_cam.getTrigMode(trig_mode);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::setExpTime(double exp_time)
{
	m_cam.setExpTime(exp_time);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::getExpTime(double& exp_time)
{
	m_cam.getExpTime(exp_time);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::setLatTime(double lat_time)
{
	//@TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::getLatTime(double& lat_time)
{
	//@TODO
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::setNbHwFrames(int nb_frames)
{
	m_cam.setNbFrames(nb_frames);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::getNbHwFrames(int& nb_frames)
{
	m_cam.getNbFrames(nb_frames);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::getValidRanges(ValidRangesType& valid_ranges)
{
	double min_time = 10e-9;
	double max_time = 10e9;
	valid_ranges.min_exp_time = min_time;
	valid_ranges.max_exp_time = max_time;
	valid_ranges.min_lat_time = min_time;
	valid_ranges.max_lat_time = max_time;
}
