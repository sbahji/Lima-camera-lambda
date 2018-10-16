#ifndef LAMBDADETINFOCTRLOBJ_H
#define LAMBDADETINFOCTRLOBJ_H

#include "lima/Debug.h"
#include "LambdaInclude.h"
#include "LambdaCompatibility.h"
#include "lima/HwDetInfoCtrlObj.h"


namespace lima {
namespace Lambda {

class Camera;
/*******************************************************************
 * \class DetInfoCtrlObj
 * \brief Control object providing Lambda detector info interface
 *******************************************************************/

class LIBLAMBDA_API DetInfoCtrlObj: public HwDetInfoCtrlObj {
	DEB_CLASS_NAMESPC(DebModCamera, "DetInfoCtrlObj", "Lambda");

public:
	DetInfoCtrlObj(Camera& cam);
	virtual ~DetInfoCtrlObj();

	virtual void getMaxImageSize(Size& max_image_size);
	virtual void getDetectorImageSize(Size& det_image_size);

	virtual void getDefImageType(ImageType& def_image_type);
	virtual void getCurrImageType(ImageType& curr_image_type);
	virtual void setCurrImageType(ImageType curr_image_type);

	virtual void getPixelSize(double& x_size, double &y_size);
	virtual void getDetectorType(std::string& det_type);
	virtual void getDetectorModel(std::string& det_model);

	virtual void registerMaxImageSizeCallback(HwMaxImageSizeCallback& cb);
	virtual void unregisterMaxImageSizeCallback(HwMaxImageSizeCallback& cb);

private:
	Camera& m_cam;
};

} // namespace Lambda
} // namespace lima

#endif // LAMBDADETINFOCTRLOBJ_H
