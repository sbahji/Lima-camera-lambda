#ifndef LAMBDACAMERA_H
#define LAMBDACAMERA_H

#include "lima/Debug.h"
#include "Lambda.h"
#include "lima/Constants.h"
#include "lima/HwBufferMgr.h"
#include "lima/ThreadUtils.h"
//#include <fsdetector/lambda/LambdaInterface.h>

#include <libxsp.h>
class BufferCtrlObj;

namespace lima
{
namespace Lambda
{
  using namespace xsp;
  using namespace xsp::lambda;
  // using namespace DetLambdaNS;
  typedef unique_ptr<xsp::System> uptr_sys;
  typedef shared_ptr<xsp::lambda::Detector> sptr_det;
  typedef shared_ptr<xsp::Receiver> sptr_recv;
  
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

	Camera(std::string& config_path);
	~Camera();
	
	//LambdaInterface *m_objDetSys;
	uptr_sys libxsp_system;
	sptr_det detector;
	sptr_recv receiver;
	
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
	unsigned short getDistortionCorrection();

	//-- Synch control object

	TrigMode m_trigger_mode;
	void setTrigMode(TrigMode  mode);
	void getTrigMode(TrigMode& mode);

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

private:
	class CameraThread: public CmdThread
	{
	    DEB_CLASS_NAMESPC(DebModCamera, "CameraThread", "Lambda");
	public:
		enum
		{ // Status
			Ready = MaxThreadStatus, Exposure, Readout, Latency,
		};

		enum
		{ // Cmd
			StartAcq = MaxThreadCmd, StopAcq,
		};

		CameraThread(Camera& cam);

		virtual void start();
		bool m_force_stop;
		FrameStatusCode m_shFrameErrorCode;

	protected:
		virtual void init();
		virtual void execCmd(int cmd);
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
	
	
        Size m_size;
	int m_shutter_mode;
	int m_int_acq_mode;
	
	int *m_frame;
	short *m_sframe;

	// Buffer control object
	SoftBufferCtrlObj m_bufferCtrlObj;

	bool m_bBuildInCompressor;
	
	/* main acquisition thread*/
	CameraThread 	m_thread;
	int 			m_acq_frame_nb;
	
};
}
}
#endif
