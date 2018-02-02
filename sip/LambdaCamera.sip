namespace Lambda
{
  class Camera
  {
%TypeHeaderCode
#include <LambdaCamera.h>
%End
  public:
    Camera(std::string& config_path);
    ~Camera();
      
    int getNbAcquiredFrames();

    void getImageSize(Size& size /Out/);
    void getImageType(ImageType& type /Out/);
	
   // -- Buffer control object
   HwBufferCtrlObj* getBufferCtrlObj();
	
    void setNbFrames(int );
    void getNbFrames(int&);
	      
    void 	startAcq();	
    void 	stopAcq();
    void        prepareAcq();
    void	reset();

    void getTrigMode(TrigMode &mode /Out/);
    void setTrigMode(TrigMode mode);

    float     getTemperature(); 
    float     getTemperatureSetPoint();
    void      setTemperatureSetPoint( float);
    unsigned short   getDistortionCorrection();

    std::string  getInternalAcqMode();
    void         setInternalAcqMode(std::string );
	
  };
};
