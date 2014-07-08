#include "StereoCameraAccess.h"

InterfaceStereoCamera *StereoCameraAccess::CreateStereoCamera()
{
	return new StereoCamera();
}