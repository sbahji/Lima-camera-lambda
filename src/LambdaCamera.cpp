#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h> 
#include <netdb.h>

#include "lima/Exceptions.h"

#include "LambdaCamera.h"

#include <fsdetector/lambda/LambdaSysImpl.h>

using namespace lima;
using namespace lima::Lambda;

//---------------------------------------------------------------------------------------
//! Camera::CameraThread::CameraThread()
//---------------------------------------------------------------------------------------
Camera::CameraThread::CameraThread(Camera& cam)
  : m_cam(&cam)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "CameraThread::CameraThread - BEGIN";
	m_cam->m_acq_frame_nb = 0;
	m_force_stop = false;
	m_shFrameErrorCode = 0;
	DEB_TRACE() << "CameraThread::CameraThread - END";
}

//---------------------------------------------------------------------------------------
//! Camera::CameraThread::start()
//---------------------------------------------------------------------------------------
void Camera::CameraThread::start()
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "CameraThread::start - BEGIN";
	CmdThread::start();
	waitStatus(Ready);
	DEB_TRACE() << "CameraThread::start - END";
}

//---------------------------------------------------------------------------------------
//! Camera::CameraThread::init()
//---------------------------------------------------------------------------------------
void Camera::CameraThread::init()
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "CameraThread::init - BEGIN";
	setStatus(Ready);
	DEB_TRACE() << "CameraThread::init - END";
}

/************************************************************************
 * \brief aborts the thread
 ************************************************************************/
void  Camera::CameraThread::abort()
{
	DEB_MEMBER_FUNCT();
	CmdThread::abort();
	DEB_TRACE() << "DONE";
}

//---------------------------------------------------------------------------------------
//! Camera::CameraThread::CheckImageValid()
//---------------------------------------------------------------------------------------
bool Camera::CameraThread::checkImageValid(void * tData, int lFrameNo,short shErrCode)
{
	//image invalid
	return !((tData == NULL)  && (lFrameNo == -1)  && (shErrCode == -1));
}

//---------------------------------------------------------------------------------------
//! Camera::CameraThread::execCmd()
//---------------------------------------------------------------------------------------
void Camera::CameraThread::execCmd(int cmd)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "CameraThread::execCmd - BEGIN";
	int status = getStatus();

	try
	{
		switch (cmd)
		{
			case StartAcq:
				if (status != Ready)
					throw LIMA_HW_EXC(InvalidValue, "Not Ready to StartAcq");
				execStartAcq();
				break;
		}
	}
	catch (...)
	{
	}

	DEB_TRACE() << "CameraThread::execCmd - END";
}

//---------------------------------------------------------------------------------------
//! Camera::CameraThread::execStartAcq()
//---------------------------------------------------------------------------------------
void Camera::CameraThread::execStartAcq()
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraThread::execStartAcq - BEGIN";

    const double start_sleep_time_sec = 0.310; // sleep the thread in seconds (310ms)
    const double no_frame_sleep_time_sec = 0.010; // sleep the thread in seconds (10ms)

    StdBufferCbMgr& buffer_mgr = m_cam->m_bufferCtrlObj.getBuffer();

    int acq_frame_nb;
    int nb_frames = m_cam->m_nb_frames;

    // get the compression enabled value to initialize m_bBuildInCompressor
    {
        bool compression_enabled;
        int comp_level;

        m_cam->getCompressionEnabled(compression_enabled, comp_level);
        m_cam->m_bBuildInCompressor = compression_enabled;
    }

    // start acquisition
    m_cam->m_objDetSys->StartImaging();

    // manage start delay if external trigger
    {
        TrigMode trig_mode;
	    m_cam->getTrigMode(trig_mode);

        if(trig_mode != IntTrig)
        {
            lima::Sleep(start_sleep_time_sec); // sleep the thread in seconds to be sure the camera can handle the first trigger
        }
    }

    setStatus(Exposure);
    buffer_mgr.setStartTimestamp(Timestamp::now());

    m_cam->m_acq_frame_nb = 0;
    acq_frame_nb = 0;

    bool continueAcq = true;
    
    while(continueAcq && (!m_cam->m_nb_frames || m_cam->m_acq_frame_nb < m_cam->m_nb_frames))
    {
        if(m_cam->m_objDetSys->GetQueueDepth()>0)
        {
            short* ptrshData;
            int* ptrnData;
            char* ptrchData;
            bool bValid;
            int nDataLength;
            int m_nSizeX;
            int m_nSizeY;
            int m_nDepth;
            int lambda_frame_nb;
            ImageType image_type;

            if(m_cam->m_bBuildInCompressor)
            {
                ptrchData = m_cam->m_objDetSys->GetCompressedData(acq_frame_nb, m_shFrameErrorCode, nDataLength);

                // stop acquisition
                m_cam->m_objDetSys->StopImaging();

                setStatus(Fault);

                std::stringstream tempStream;
                tempStream << "Precompressed data mode is not supported by this device!";
                REPORT_EVENT(tempStream.str());

                return;
            } 
            else 
            {  
                bool frame_was_lost = false;

                //decoded image,without pre-compression
                m_cam->m_objDetSys->GetImageFormat(m_nSizeX,m_nSizeY,m_nDepth);
                m_cam->getImageType(image_type);
                FrameDim frame_dim( m_nSizeX, m_nSizeY, image_type);

                // short
                if(image_type == Bpp12)
                { 
                    ptrshData = m_cam->m_objDetSys->GetDecodedImageShort(lambda_frame_nb,  m_shFrameErrorCode);

                    setStatus(Readout);

                    if(checkImageValid(ptrshData,lambda_frame_nb, m_shFrameErrorCode))
                    {
                        //check if image is error
                        if(m_shFrameErrorCode == 0)
                        {
                            void *ptr = buffer_mgr.getFrameBufferPtr(acq_frame_nb);
                            memcpy((short *)ptr, (short *)ptrshData, frame_dim.getMemSize()); //we need a nb of BYTES .
                        }
                        else
                        {
                            frame_was_lost = true;
                        }
                    }
                    else
                    {
                        // not an error but we need to recall the get image method latter
                        continue;
                    }
                }
                else 
                // int
                if(image_type == Bpp24)
                { 
                    // get the address of the image in memory
                    ptrnData = m_cam->m_objDetSys->GetDecodedImageInt(lambda_frame_nb,  m_shFrameErrorCode);

                    setStatus(Readout);

                    if(checkImageValid(ptrnData,lambda_frame_nb, m_shFrameErrorCode))
                    {
                        //check if image is error
                        if(m_shFrameErrorCode == 0)
                        {
                            void *ptr = buffer_mgr.getFrameBufferPtr(acq_frame_nb);
                            memcpy((int *)ptr, (int *)ptrnData, frame_dim.getMemSize()); //we need a nb of BYTES .
                        }
                        else
                        {
                            frame_was_lost = true;
                        }
                    }
                    else
                    {
                        // not an error but we need to recall the get image method latter
                        continue;
                    }
                }

                if(frame_was_lost)
                {
                    // stop acquisition
                    m_cam->m_objDetSys->StopImaging();

                    setStatus(Fault);

                    std::stringstream tempStream;
                    tempStream << "Frame lost during acquisition! Acquisition was aborted. error code=" << m_shFrameErrorCode;
                    REPORT_EVENT(tempStream.str());
                    return;
                }
            }

            buffer_mgr.setStartTimestamp(Timestamp::now());

            DEB_TRACE() << "Declare a new Frame Ready.";
            HwFrameInfoType frame_info;
            frame_info.acq_frame_nb = m_cam->m_acq_frame_nb;
            buffer_mgr.newFrameReady(frame_info);

            acq_frame_nb++;
            m_cam->m_acq_frame_nb = acq_frame_nb;
        }
        else
        {
            lima::Sleep(no_frame_sleep_time_sec); // sleep the thread in seconds
        }

        if(m_force_stop)
        {
            continueAcq = false;
            m_force_stop = false;
            break;
        }

    } /* End while */

    // stop acquisition
    m_cam->m_objDetSys->StopImaging();
 
    /*
    if (m_cam->m_frame){
    delete[] m_cam->m_frame;
    m_cam->m_frame = 0;
    }
    if (m_cam->m_sframe){
    delete[] m_cam->m_sframe;
    m_cam->m_sframe = 0;
    }
    */

    setStatus(Ready);

    DEB_TRACE() << "CameraThread::execStartAcq - END";
}

//---------------------------------------------------------------------------------------
//! Camera::Camera()
//---------------------------------------------------------------------------------------
Camera::Camera(std::string& config_path, bool distortion_correction):
m_thread(*this),
m_roi_s1(0),
m_roi_s2(2047),
m_roi_sbin(1),
m_roi_p1(0),
m_roi_p2(2047),
m_roi_pbin(1),
m_nb_frames(1),
m_exposure(1.0),
m_int_acq_mode(0),
m_bufferCtrlObj()
{
	DEB_CONSTRUCTOR();
	DEB_TRACE() << "Camera::Camera";
	m_bBuildInCompressor = false;
	m_objDetSys = new LambdaSysImpl(config_path);
	
    setDistortionCorrection(distortion_correction);

	m_thread.start();
}

//---------------------------------------------------------------------------------------
//! Camera::~Camera()
//---------------------------------------------------------------------------------------
Camera::~Camera()
{
	DEB_DESTRUCTOR();
	DEB_TRACE() << "Camera::~Camera";
	stopAcq();
}

//---------------------------------------------------------------------------------------
//! Camera::getStatus()
//---------------------------------------------------------------------------------------
Camera::Status Camera::getStatus()
{
	DEB_MEMBER_FUNCT();

	int thread_status = m_thread.getStatus();

	DEB_RETURN() << DEB_VAR1(thread_status);
	
	switch (thread_status)
	{
		case CameraThread::Ready:
			return Camera::Ready;
		case CameraThread::Exposure:
			return Camera::Exposure;
		case CameraThread::Readout:
			return Camera::Readout;
		case CameraThread::Latency:
			return Camera::Latency;
		case CameraThread::Fault:
			return Camera::Fault;
		default:
			throw LIMA_HW_EXC(Error, "Invalid thread status");
	}
}

//---------------------------------------------------------------------------------------
//! Camera::setNbFrames()
//---------------------------------------------------------------------------------------
void Camera::setNbFrames(int nb_frames)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setNbFrames - " << DEB_VAR1(nb_frames);
	if (nb_frames < 0)
		throw LIMA_HW_EXC(InvalidValue, "Invalid nb of frames");
	  
	if(nb_frames == 0){
	  m_objDetSys->SetNImages(1000000);
	} else {
	  m_objDetSys->SetNImages(nb_frames);
	}
	m_nb_frames = nb_frames;
}

//---------------------------------------------------------------------------------------
//! Camera::getNbFrames()
//---------------------------------------------------------------------------------------
void Camera::getNbFrames(int& nb_frames)
{
	DEB_MEMBER_FUNCT();
	DEB_RETURN() << DEB_VAR1(m_nb_frames);

	nb_frames = m_nb_frames;
}

//---------------------------------------------------------------------------------------
//! Camera::getDetectorModel()
//---------------------------------------------------------------------------------------
void Camera::getDetectorModel(std::string& model)
{
	DEB_MEMBER_FUNCT();
	stringstream ss;
	ss <<"ModId "<< m_objDetSys->GetModuleID()
	    <<" - Firmware "<<m_objDetSys->GetFirmwareVersion()
	    <<" - Detector Core "<<m_objDetSys->GetDetCoreVersion()
	    <<" - Liblambda "<<m_objDetSys->GetLibLambdaVersion();
	model = ss.str();
}

//---------------------------------------------------------------------------------------
//! Camera::getNbAcquiredFrames()
//---------------------------------------------------------------------------------------
int Camera::getNbAcquiredFrames()
{
	return m_acq_frame_nb;
}

//---------------------------------------------------------------------------------------
//! Camera::prepareAcq()
//---------------------------------------------------------------------------------------
void Camera::prepareAcq()
{
	DEB_MEMBER_FUNCT();
}

//---------------------------------------------------------------------------------------
//! Camera::startAcq()
//---------------------------------------------------------------------------------------
void Camera::startAcq()
{
	DEB_MEMBER_FUNCT();

	m_thread.m_force_stop = false;
	m_acq_frame_nb = 0;

	m_thread.sendCmd(CameraThread::StartAcq);
	m_thread.waitNotStatus(CameraThread::Ready);
}

//---------------------------------------------------------------------------------------
//! Camera::stopAcq()
//---------------------------------------------------------------------------------------
void Camera::stopAcq()
{
	DEB_MEMBER_FUNCT();

	DEB_TRACE() << "executing StopAcq command...";

	if(m_thread.getStatus() != CameraThread::Fault)
	{
		m_thread.m_force_stop = true;

		// Waiting for thread to finish
		m_thread.waitStatus(CameraThread::Ready);
	}

	// thread in error
	if(m_thread.getStatus() == CameraThread::Fault)
	{
		// aborting & restart the thread
		m_thread.abort();
	}
}

//---------------------------------------------------------------------------------------
//! Camera::reset()
//---------------------------------------------------------------------------------------
void Camera::reset()
{
	DEB_MEMBER_FUNCT();
	//@todo maybe something to do!
}

//---------------------------------------------------------------------------------------
//! Camera::getExpTime()
//---------------------------------------------------------------------------------------
void Camera::getExpTime(double& exp_time)
{
	DEB_MEMBER_FUNCT();
	//	AutoMutex aLock(m_cond.mutex());
	exp_time = m_exposure / 1E3;//the lima standard unit is second AND default detector unit is ms
	DEB_RETURN() << DEB_VAR1(exp_time);
}

//---------------------------------------------------------------------------------------
//! Camera::setExpTime()
//---------------------------------------------------------------------------------------
void Camera::setExpTime(double  exp_time)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setExpTime - " << DEB_VAR1(exp_time);

	m_exposure = exp_time * 1E3;//default detector unit is ms
	m_objDetSys->SetShutterTime(m_exposure);
}

//---------------------------------------------------------------------------------------
//! Camera::getLatencyTime()
//---------------------------------------------------------------------------------------
void Camera::getLatencyTime(double & latency_time)
{
	DEB_MEMBER_FUNCT();
	latency_time = m_objDetSys->GetDelayTime() / 1E3;//the lima standard unit is second AND default detector unit is ms
	DEB_RETURN() << DEB_VAR1(latency_time);
}

//---------------------------------------------------------------------------------------
//! Camera::setLatencyTime()
//---------------------------------------------------------------------------------------
void Camera::setLatencyTime(double  latency_time)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setLatencyTime - " << DEB_VAR1(latency_time);

	latency_time = latency_time * 1E3;//default detector unit is ms
	m_objDetSys->SetDelayTime(latency_time);
}

//---------------------------------------------------------------------------------------
//! Camera::setTrigMode()
//---------------------------------------------------------------------------------------
void Camera::setTrigMode(TrigMode mode)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setTrigMode - " << DEB_VAR1(mode);
	DEB_PARAM() << DEB_VAR1(mode);

	switch (mode) 
    {
	// Internal trigger
    case IntTrig:
    case IntTrigMult:
	  m_objDetSys->SetTriggerMode(0); 
	  break;

	// External trigger. Once detector receives trigger, it takes predefined image numbers.
    case ExtTrigSingle:
	  m_objDetSys->SetTriggerMode(1); 
	  break;
	
	// External trigger. Each trigger pulse takes one image.
	case ExtTrigMult:
    case ExtGate:
	  m_objDetSys->SetTriggerMode(2); 
	  break;
	
	case ExtStartStop:
	case ExtTrigReadout:
	default:
		THROW_HW_ERROR(Error) << "Cannot change the Trigger Mode of the camera, this mode is not managed!";
		break;
	}
}

//---------------------------------------------------------------------------------------
//! Camera::getTrigMode()
//---------------------------------------------------------------------------------------
void Camera::getTrigMode(TrigMode& mode)
{
	DEB_MEMBER_FUNCT();

    int16 trigger_mode = m_objDetSys->GetTriggerMode();

	// Internal trigger
    if(trigger_mode == 0)
    {
        mode = IntTrig;
    }
    else
	// External trigger. Once detector receives trigger, it takes predefined image numbers.
    if(trigger_mode == 1)
    {
        mode = ExtTrigSingle;
    }
    else
	// External trigger. Each trigger pulse takes one image.
    if(trigger_mode == 2)
    {
        ImageType image_type;
        getImageType(image_type);

        if(image_type == Bpp12)
        {
            mode = ExtGate;
        }
        else
        if(image_type == Bpp24)
        {
            mode = ExtTrigMult;
        }
    }
    else
    {
		THROW_HW_ERROR(Error) << "Unmanaged trigger Mode from the camera!";
    }
    
	DEB_RETURN() << DEB_VAR1(mode);
}

//-----------------------------------------------------
//! Camera::checkTrigMode()
//-----------------------------------------------------
bool Camera::checkTrigMode(TrigMode trig_mode)
{
	bool valid_mode = false;
	ImageType image_type;

        switch (trig_mode)
	{
		case IntTrig:
		case ExtTrigSingle:
			valid_mode = true;
		break;

		case ExtTrigMult:
			getImageType(image_type);
			valid_mode = (image_type == Bpp24);
		break;

		case ExtGate:
			getImageType(image_type);
			valid_mode = (image_type == Bpp12);
		break;

		default:
			valid_mode = false;
	}

    return valid_mode;
}

//---------------------------------------------------------------------------------------
//! Camera::setShutterMode()
//---------------------------------------------------------------------------------------
void Camera::setShutterMode(int mode)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setShutterMode - " << DEB_VAR1(mode);
	DEB_PARAM() << DEB_VAR1(mode);
}

//---------------------------------------------------------------------------------------
//! Camera::getShutterMode()
//---------------------------------------------------------------------------------------
void Camera::getShutterMode(int& mode)
{
	DEB_MEMBER_FUNCT();
	mode = m_shutter_mode;
	DEB_RETURN() << DEB_VAR1(mode);
}

//---------------------------------------------------------------------------------------
//! Camera::getTemperature()
//---------------------------------------------------------------------------------------
double Camera::getTemperature()
{
	DEB_MEMBER_FUNCT();
}

//---------------------------------------------------------------------------------------
//! Camera::getTemperatureSetPoint()
//---------------------------------------------------------------------------------------
double Camera::getTemperatureSetPoint()
{
	DEB_MEMBER_FUNCT();
}

//---------------------------------------------------------------------------------------
//! Camera::setTemperatureSetPoint()
//---------------------------------------------------------------------------------------
void Camera::setTemperatureSetPoint(double temperature)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setTemperatureSetPoint - " << DEB_VAR1(temperature);

}

//---------------------------------------------------------------------------------------
//! Camera::getInternalAcqMode()
//---------------------------------------------------------------------------------------
std::string Camera::getInternalAcqMode()
{
	DEB_MEMBER_FUNCT();
	std::string mode = "UNKNOWN";
	if (m_int_acq_mode == 0)
	{
		DEB_RETURN() << DEB_VAR1("STANDARD");
		mode = "STANDARD";
	}
	else if (m_int_acq_mode == 1)
	{
		DEB_RETURN() << DEB_VAR1("CONTINUOUS");
		mode = "CONTINUOUS";
	}
	else if (m_int_acq_mode == 2)
	{
		DEB_RETURN() << DEB_VAR1("FOCUS");
		mode = "FOCUS";
	}
	return mode;
}

//---------------------------------------------------------------------------------------
//! Camera::setInternalAcqMode()
//---------------------------------------------------------------------------------------
void Camera::setInternalAcqMode(std::string mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(mode);

	if (mode == "STANDARD")
		m_int_acq_mode = 0;
	else if (mode == "CONTINUOUS")
		m_int_acq_mode = 1;
	else if (mode == "FOCUS")
		m_int_acq_mode = 2;
	else
		THROW_HW_ERROR(Error) << "Incorrect Internal Acquisition mode !";
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setImageType(ImageType type)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setImageType - " << DEB_VAR1(type);

    switch(type)
    {
    case Bpp12 : 
        setOperationMode("ContinuousReadWrite");
        break;

    case Bpp24 : 
        setOperationMode("TwentyFourBit");
        break;

    default:
        THROW_HW_ERROR(Error) << "This pixel format of the camera is not managed, only 12 bits and 24 bits formats are already managed!";
    break;
    }
}

//---------------------------------------------------------------------------------------
//! Camera::getImageType()
//---------------------------------------------------------------------------------------
void Camera::getImageType(ImageType& type)
{
	DEB_MEMBER_FUNCT();
	int m_nSizeX;
	int m_nSizeY;
	int m_nDepth;
	
	m_objDetSys->GetImageFormat(m_nSizeX,m_nSizeY,m_nDepth);
	
	if(m_nDepth == 12)
	  type = Bpp12; //short
	else
	if(m_nDepth == 24)
	  type = Bpp24; //int
    else
	  THROW_HW_ERROR(Error) << "Unmanaged image depth from the camera!";
}

//---------------------------------------------------------------------------------------
//! Camera::setBin()
//---------------------------------------------------------------------------------------
void Camera::setBin(const Bin& bin)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setBin";
	DEB_PARAM() << DEB_VAR1(bin);

	m_roi_sbin = bin.getX();
	m_roi_pbin = bin.getY();
}

//---------------------------------------------------------------------------------------
//! Camera::getBin()
//---------------------------------------------------------------------------------------
void Camera::getBin(Bin& bin)
{
	DEB_MEMBER_FUNCT();
	Bin tmp_bin(m_roi_sbin, m_roi_pbin);
	bin = tmp_bin;

	DEB_RETURN() << DEB_VAR1(bin);
}

//---------------------------------------------------------------------------------------
//! Camera::checkBin()
//---------------------------------------------------------------------------------------
void Camera::checkBin(Bin& bin)
{

	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::checkBin";
	DEB_PARAM() << DEB_VAR1(bin);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::checkRoi";
	DEB_PARAM() << DEB_VAR1(set_roi);
	hw_roi = set_roi;

	DEB_RETURN() << DEB_VAR1(hw_roi);
}

//---------------------------------------------------------------------------------------
//! Camera::getRoi()
//---------------------------------------------------------------------------------------
void Camera::getRoi(Roi& hw_roi)
{
	DEB_MEMBER_FUNCT();
	Point point1(m_roi_s1, m_roi_p1);
	Point point2(m_roi_s2, m_roi_p2);
	Roi tmp_roi(point1, point2);

	hw_roi = tmp_roi;

	DEB_RETURN() << DEB_VAR1(hw_roi);
}

//---------------------------------------------------------------------------------------
//! Camera::setRoi()
//---------------------------------------------------------------------------------------
void Camera::setRoi(const Roi& set_roi)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setRoi";
	DEB_PARAM() << DEB_VAR1(set_roi);
	if (!set_roi.isActive())
	{
		DEB_TRACE() << "Roi is not Enabled";
	}
	else
	{
		//To avoid a double binning, API pvcam apply Roi & Binning together AND Lima Too !		
		Bin aBin;
		getBin(aBin);
		Roi UnbinnedRoi = set_roi.getUnbinned(aBin);
		Point tmp_top = UnbinnedRoi.getTopLeft();
		////

		m_roi_s1 = tmp_top.x;
		m_roi_p1 = tmp_top.y;

		Point tmp_bottom = UnbinnedRoi.getBottomRight();

		m_roi_s2 = tmp_bottom.x;
		m_roi_p2 = tmp_bottom.y;
	}
}

void Camera::getImageSize(Size& size) {
	DEB_MEMBER_FUNCT();

	int m_nSizeX;
	int m_nSizeY;
	int m_nDepth;
	
	m_objDetSys->GetImageFormat(m_nSizeX,m_nSizeY,m_nDepth);
	size = Size(m_nSizeX,m_nSizeY);
}

HwBufferCtrlObj* Camera::getBufferCtrlObj() {
    return &m_bufferCtrlObj;
}

// 0 : no distortion correction
// 1 : division
bool Camera::getDistortionCorrection()
{
  return (m_objDetSys->GetDistortionCorrecttionMethod() == 1);
}

void Camera::setDistortionCorrection(bool distortion_correction)
{
    int32 distortionCorrection = (distortion_correction) ? 1 : 0;
    m_objDetSys->SetDistortionCorrecttionMethod(distortionCorrection);
}

/**
 * @brief enable compression
 *        Note: if compression is enabled, please use GetCompressedData method
 * @param compression_enabled;true: use compression. false: do not compression
 * @param comp_level compression level
 */
void Camera::setCompressionEnabled(bool compression_enabled, int comp_level)
{
    m_objDetSys->SetCompressionEnabled(compression_enabled, static_cast<int32>(comp_level));
}

/**
 * @brief get compression status 
 * @param compression_enabled;true: use compression. false: do not compression
 * @param comp_level compression level
 */
void Camera::getCompressionEnabled(bool & compression_enabled, int & comp_level)
{
    int32 temp_comp_level;
    m_objDetSys->GetCompressionEnabled(compression_enabled, temp_comp_level);
    comp_level = static_cast<int>(temp_comp_level);
}

/**
 * @brief set the operation mode
 * @param operation_mode operation mode
 */
void Camera::setOperationMode(std::string operation_mode)
{
    m_objDetSys->SetOperationMode(operation_mode);
}

/**
 * @brief get the operation mode
 * @return operation mode
 */
std::string Camera::getOperationMode()
{
    return m_objDetSys->GetOperationMode();
}

// Path to the main directory where configuration files are contained.
std::string Camera::getConfigFilePath()
{
    return m_objDetSys->GetConfigFilePath();
}

float Camera::getReadoutTimeMs()
{
    float readout_time;
    std::string operation_mode = Camera::getOperationMode();
    
    if(operation_mode == "TwentyFourBit")
    {
        readout_time = 1.0f;
    }
    else
    {
        readout_time = 0.0f;
    }

    return readout_time;
}


// Sets threshold number threshold_no to energy (in keV).
// In standard operating modes, only threshold no. 0 is used.
void Camera::setThresholdEnergy(int threshold_no, float energy)
{
    m_objDetSys->SetThreshold(static_cast<int32>(threshold_no), energy / 1000.0f);
} 

float Camera::getThresholdEnergy(int threshold_no)
{
    return m_objDetSys->GetThreshold(static_cast<int32>(threshold_no)) * 1000.0f;
}

bool Camera::getBurstMode()
{
    return m_objDetSys->GetBurstMode();
}

/*******************************************************************
 * \brief Gets the Lima event control object
 * \return event control object pointer
*******************************************************************/
HwEventCtrlObj* Camera::getEventCtrlObj()
{
	return &m_event_ctrl_obj;
}
