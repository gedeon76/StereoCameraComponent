#include "StereoCameraAccess.h"

InterfaceStereoCamera *StereoCameraAccess::CreateStereoCamera()
{
	InterfaceStereoCamera *Obj = new StereoCamera();
	return Obj;
}