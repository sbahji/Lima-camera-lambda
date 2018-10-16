#ifndef LAMBDACAMERA_H
#define LAMBDACAMERA_H

#include "lima/Debug.h"
#include "LambdaInclude.h"
#include "lima/Constants.h"
#include "lima/HwBufferMgr.h"
#include "lima/ThreadUtils.h"
#include "lima/HwEventCtrlObj.h"

#include <fsdetector/lambda/LambdaInterface.h>

/*************************************************************************/
#define REPORT_EVENT(desc)  {   \
                                Event *my_event = new Event(Hardware,Event::Info, Event::Camera, Event::Default,desc); \
                                m_cam->getEventCtrlObj()->reportEvent(my_event); \
                            }
/*************************************************************************/

class BufferCtrlObj;

namespace lima
{
namespace Lambda
{
     using namespace DetLambdaNS;
//    class Camera
class LIBLAMBDA_API Camera
{
	DEB_CLASS_NAMESPC(DebModCamera,"Camera","Lambda");
	friend class Interface;
public:
	
	enum Status
	{
		Ready, Exposure, Readout, Latency, Fault
	};

	Camera(std::string& config_path, bool distortion_correction);
	~Camera();
	
	LambdaInterface *m_objDetSys;
	
	void getExpTime(double& exp_time);
	void setExpTime(double  exp_time);

	Status getStatus();
	int  getNbAcquiredFrames();
	
	void getDetectorModel(std::string& model);
	void startAcq();
	void stopAcq();
	void prepareAcq();
	void reset();

	void setNbFrames(int nb_frames);
	void getNbFrames(int& nb_frames);
	

	HwBufferCtrlObj* getBufferCtrlObj();
	
	double 	getTemperature();
	double 	getTemperatureSetPoint();
	void	setTemperatureSetPoint(double temperature);

	bool getDistortionCorrection();
	void setDistortionCorrection(bool distortion_correction);
    
	void setOperationMode(std::string operation_mode);
	std::string getOperationMode();

	void setCompressionEnabled(bool compression_enabled, int comp_level);
	void getCompressionEnabled(bool & compression_enabled, int & comp_level);

	std::string getConfigFilePath();

	float getReadoutTimeMs();

	void setThresholdEnergy(int threshold_no, float energy);
	float getThresholdEnergy(int threshold_no);

	bool getBurstMode();

	void getLatencyTime(double & latency_time);
	void setLatencyTime(double  latency_time);

	//-- Synch control object
	void setTrigMode(TrigMode  mode);
	void getTrigMode(TrigMode& mode);
	bool checkTrigMode(TrigMode trig_mode);

	//-- Shutter managment : TODO Shutter control object
	//---------------------------------------------------------------------------------------
	void setShutterMode(int mode);
	void getShutterMode(int& mode);
	
	std::string getInternalAcqMode();
	void setInternalAcqMode(std::string mode);


	void getImageSize(Size& size);	
	void setImageType(ImageType type);
	void getImageType(ImageType& type);
	
	void setBin(const Bin& bin);
	void getBin(Bin& bin);
	void checkBin(Bin& bin);

	void checkRoi(const Roi& set_roi, Roi& hw_roi);
	void setRoi(const Roi& set_roi);
	void getRoi(Roi& hw_roi);   

	// Gets the Lima event control object
	HwEventCtrlObj* getEventCtrlObj();

private:
	class CameraThread: public CmdThread
	{
	    DEB_CLASS_NAMESPC(DebModCamera, "CameraThread", "Lambda");
	public:
		enum
		{ // Status
			Ready = MaxThreadStatus, Exposure, Readout, Latency, Fault,
		};

		enum
		{ // Cmd
			StartAcq = MaxThreadCmd, StopAcq,
		};

		CameraThread(Camera& cam);

		virtual void start();
		
		// aborts the thread
		virtual void abort();

		bool m_force_stop;
		short m_shFrameErrorCode;

	protected:
		virtual void init();
		virtual void execCmd(int cmd);

	private:
		bool checkImageValid(void * tData, int lFrameNo,short shErrCode);

	private:
		void execStartAcq();
		void execStopAcq();
		Camera* m_cam;

	};
	friend class CameraThread;
	
	/* Related to API PvCam */
	int short m_handle; 
	char m_name[128];
	double m_exposure;

	char m_error_msg[200];
	int m_error_code;
	int m_nb_frames;

	unsigned short m_roi_s1;
	unsigned short m_roi_s2;
	unsigned short m_roi_sbin;
	unsigned short m_roi_p1;
	unsigned short m_roi_p2;
	unsigned short m_roi_pbin;
	
	int m_shutter_mode;
	int m_int_acq_mode;

	// Buffer control object
	SoftBufferCtrlObj m_bufferCtrlObj;

    // Lima event control object
    HwEventCtrlObj m_event_ctrl_obj;

	bool m_bBuildInCompressor;
	
	/* main acquisition thread*/
	CameraThread 	m_thread;
	int 			m_acq_frame_nb;
	
};
}
}
#endif
