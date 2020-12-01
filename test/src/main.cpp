
//- C++
#include <iostream>
#include <string>
#include <time.h>


//- LIMA
#include <lima/HwInterface.h>
#include <lima/CtControl.h>
#include <lima/CtAcquisition.h>
#include <lima/CtVideo.h>
#include <lima/CtImage.h>
#include <LambdaInterface.h>
#include <LambdaCamera.h>

//--------------------------------------------------------------------------------------
//waiting the status of acquisition
//--------------------------------------------------------------------------------------
void wait_while_status_is_standby(lima::CtControl& ct, lima::Lambda::Interface& hw)
{
	lima::CtControl::Status status;
	lima::HwInterface::StatusType state;

	bool is_already_displayed = false;
	while (1)
	{

		// let's take a look at the status of control & the status of the plugin
		lima::CtControl::Status ctStatus;
		ct.getStatus(ctStatus);

		switch (ctStatus.AcquisitionStatus)
		{
			case lima::AcqReady:
			{
				if (!is_already_displayed)
				{
					std::cout << "Waiting for Request ..." << std::endl;
					is_already_displayed = true;
				}
			}
				break;

			case lima::AcqRunning:
			{
				std::cout << "Acquisition is Running ..." << std::endl;
				return;
			}
				break;

			case lima::AcqConfig:
			{
				std::cout << "--> Detector is Calibrating...\n" << std::endl;
				return;
			}
				break;

			default:
			{
				std::cout << "--> Acquisition is in Fault\n" << std::endl;
				return;
			}
				break;
		}
	}
}
//--------------------------------------------------------------------------------------
//waiting the status of acquisition
//--------------------------------------------------------------------------------------
void wait_while_status_is_running(lima::CtControl& ct, lima::Lambda::Interface& hw)
{
	lima::CtControl::Status status;
	lima::HwInterface::StatusType state;

	bool is_already_displayed = false;
	while (1)
	{

		// let's take a look at the status of control & the status of the plugin
		lima::CtControl::Status ctStatus;
		ct.getStatus(ctStatus);

		switch (ctStatus.AcquisitionStatus)
		{
			case lima::AcqReady:
			{
				std::cout << "Waiting for Request ...\n" << std::endl;
				return;
			}
				break;

			case lima::AcqRunning:
			{
				if (!is_already_displayed)
				{
					std::cout << "Acquisition is Running ..." << std::endl;
					is_already_displayed = true;
				}
			}
				break;

			case lima::AcqConfig:
			{
				std::cout << "--> Detector is Calibrating...\n" << std::endl;
				return;
			}
				break;

			default:
			{
				std::cout << "--> Acquisition is in Fault\n" << std::endl;
				return;
			}
				break;
		}
	}
}

//--------------------------------------------------------------------------------------
// start a snap
//--------------------------------------------------------------------------------------
void do_snap(lima::CtControl& ct, lima::Lambda::Interface& hw)
{
	//- prepare acqusition
	std::cout << "prepareAcq" << std::endl;
	ct.prepareAcq();

	//- start acqusition
	std::cout << "startAcq" << std::endl;
	ct.startAcq();

	//- waiting while state is standby ...	
	wait_while_status_is_standby(ct, hw);

	clock_t start = clock();
	//- waiting while state is running ...
	wait_while_status_is_running(ct, hw);

	clock_t end = clock();
	double millis = (end - start) / 1000;
	std::cout << "Elapsed time  = " << millis << " (ms)" << std::endl;
	std::cout << "============================================\n" << std::endl;
}

//--------------------------------------------------------------------------------------
//test main:

//- 1st argument is the exposure_time (in milli seconds)
//- 2nd argument is the latency_time (in milli seconds)
//- 3rd argument is the latency_time (in milli seconds)
//- 4th argument is the nb. of frames to acquire
//- 5th argument is the nb. of snap to do
//- 6th argument is the Bpp
//--------------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	std::cout << "============================================" << std::endl;
	std::cout << "Usage :./ds_TestLimaLambda exp lat expacc nbframes nbsnap bpp" << std::endl << std::endl;

	try
	{
		double exposure_time			= 500;				//default value is 1000 ms
		double latency_time				= 0.0;				//default value is 0 ms
		double acc_max_exposure_time	= 100;				//default value is 100 ms
		int nb_frames					= 1;				//default value is 10
		int nb_snap						= 1;                //default value
		unsigned bpp					= 24; 

		//read args of main 
		switch (argc)
		{
			case 2:
			{
				std::istringstream argExposure(argv[1]);
				argExposure >> exposure_time;
			}
				break;

			case 3:
			{
				std::istringstream argExposure(argv[1]);
				argExposure >> exposure_time;

				std::istringstream argLatency(argv[2]);
				argLatency >> latency_time;
			}
				break;

			case 4:
			{
				std::istringstream argExposure(argv[1]);
				argExposure >> exposure_time;

				std::istringstream argLatency(argv[2]);
				argLatency >> latency_time;

				std::istringstream argAccExposure(argv[3]);
				argAccExposure >> acc_max_exposure_time;
			}
				break;

			case 5:
			{
				std::istringstream argExposure(argv[1]);
				argExposure >> exposure_time;

				std::istringstream argLatency(argv[2]);
				argLatency >> latency_time;

				std::istringstream argAccExposure(argv[3]);
				argAccExposure >> acc_max_exposure_time;

				std::istringstream argNbSnap(argv[4]);
				argNbSnap >> nb_snap;
			}
				break;

			case 6:
			{
				std::istringstream argExposure(argv[1]);
				argExposure >> exposure_time;

				std::istringstream argLatency(argv[2]);
				argLatency >> latency_time;

				std::istringstream argAccExposure(argv[3]);
				argAccExposure >> acc_max_exposure_time;

				std::istringstream argNbFrames(argv[4]);
				argNbFrames >> nb_frames;

				std::istringstream argNbSnap(argv[5]);
				argNbSnap >> nb_snap;

			}
				break;

			case 7:
			{
				std::istringstream argExposure(argv[1]);
				argExposure >> exposure_time;

				std::istringstream argLatency(argv[2]);
				argLatency >> latency_time;

				std::istringstream argAccExposure(argv[3]);
				argAccExposure >> acc_max_exposure_time;

				std::istringstream argNbFrames(argv[4]);
				argNbFrames >> nb_frames;

				std::istringstream argNbSnap(argv[5]);
				argNbSnap >> nb_snap;

				std::istringstream argBpp(argv[6]);
				argBpp >> bpp;

			}
				break;
		}

		//        
		std::cout << "============================================" << std::endl;
		std::cout << "exposure_time          = " << exposure_time << std::endl;
		std::cout << "latency_time           = " << latency_time << std::endl;
		std::cout << "acc_max_exposure_time  = " << acc_max_exposure_time << std::endl;
		std::cout << "nb_frames              = " << nb_frames << std::endl;
		std::cout << "nb_snap                = " << nb_snap << std::endl;
		std::cout << "bpp                    = " << bpp << std::endl;
		std::cout << "============================================" << std::endl;

		//initialize Lambda::Camera objects & Lima Objects
		std::cout << "Create Camera Object" << std::endl;
        std::string config_file_path = "/opt/xsp/config";
        bool distortion_correction = true;
		lima::Lambda::Camera my_camera(config_file_path, distortion_correction);

		std::cout << "Create Interface Object" << std::endl;
		lima::Lambda::Interface my_interface(my_camera);

		std::cout << "Create CtControl Object" << std::endl;
		lima::CtControl my_control(&my_interface);

		std::cout << "============================================" << std::endl;
		lima::HwDetInfoCtrlObj *hw_det_info;
		my_interface.getHwCtrlObj(hw_det_info);

		switch (bpp)
		{
			case 8:
				//- Set imageType = Bpp16
				std::cout << "Set imageType \t= " << bpp << std::endl;
				hw_det_info->setCurrImageType(lima::Bpp8);
				break;

			case 12:
				//- Set imageType = Bpp16
				std::cout << "Set imageType \t= " << bpp << std::endl;
				hw_det_info->setCurrImageType(lima::Bpp12);
				break;

			case 16:
				//- Set imageType = Bpp16
				std::cout << "Set imageType \t= " << bpp << std::endl;
				hw_det_info->setCurrImageType(lima::Bpp16);
				break;
			case 32:
				//- Set imageType = Bpp16
				std::cout << "Set imageType \t= " << bpp << std::endl;
				hw_det_info->setCurrImageType(lima::Bpp32);
				break;

			default:
				//- Error
				std::cout << "Don't Set imageType !" << std::endl;
				break;				
		}

		//- Set Roi = (0,0,MaxWidth,MaxHeight)
		lima::Size size;
		hw_det_info->getMaxImageSize(size);
		lima::Roi myRoi(0, 0, size.getWidth(), size.getHeight());
		std::cout << "Set Roi \t= (" << 0 << "," << 0 << "," << size.getWidth() << "," << size.getHeight() << ")" << std::endl;
		my_control.image()->setRoi(myRoi);

		//- Set Bin = (1,1)
		std::cout << "Set Bin \t= (1,1)" << std::endl;
		lima::Bin myBin(1, 1);
		my_control.image()->setBin(myBin);

		//- Set exposure
		std::cout << "Set exposure \t= " << exposure_time << " (ms)" << std::endl;
		my_control.acquisition()->setAcqExpoTime(exposure_time / 1000.0); //convert exposure_time to sec
		my_control.video()->setExposure(exposure_time / 1000.0); //convert exposure_time to sec

		//- Set latency
		std::cout << "Set latency \t= " << latency_time << " (ms)" << std::endl;
		my_control.acquisition()->setLatencyTime(latency_time / 1000.0);  //convert latency_time to sec

        //- Set Acc exposure
		std::cout << "Set Acc exposure= " << acc_max_exposure_time << " (ms)" << std::endl;
		my_control.acquisition()->setAccMaxExpoTime(acc_max_exposure_time / 1000.0); //convert exposure_time to sec
		
		//- Set nbFrames
		std::cout << "Set nbFrames \t= " << nb_frames << std::endl;
		my_control.acquisition()->setAcqNbFrames(nb_frames);

		std::cout << "\n" << std::endl;

		//start nb_snap acquisition of n frame
		for (unsigned i = 0; i < nb_snap; i++)
		{
			//- Tests of AcqMode:
			std::cout << "============================================" << std::endl;
			std::cout << "-	 	AcqMode Tests		  -" << std::endl;
			std::cout << "============================================" << std::endl;
			lima::AcqMode an_acq_mode;
			my_control.acquisition()->getAcqMode(an_acq_mode);
			std::cout << "Setting AcqMode to SINGLE" << std::endl;
			my_control.acquisition()->setAcqMode(lima::Single);
			//			std::cout << "Resetting Status(False)..." << std::endl;
			//			my_control.resetStatus(false);

			std::cout << "Snapping ..." << std::endl;
			do_snap(my_control, my_interface);

			std::cout << "Setting AcqMode to ACCUMULATION" << std::endl;
			my_control.acquisition()->setAcqMode(lima::Accumulation);
			//			std::cout << "Resetting Status(False)" << std::endl;
			//			my_control.resetStatus(false);
			std::cout << "Snapping ..." << std::endl;
			do_snap(my_control, my_interface);
		}
	}
	catch (lima::Exception e)
	{
		std::cerr << "LIMA Exception : " << e << std::endl;
	}

    catch (...)
	{
		std::cerr << "Unknown Exception" << std::endl;
	}

	return 0;
}
//--------------------------------------------------------------------------------------
