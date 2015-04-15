# try to find the 3Dpointer libraries
#
#	author:		Henry Portilla
#	date:		oct.24.2014

# Once done this will define
# 3DPOINTER_FOUND			- the 3dpointer libraries has been found
# 3DPOINTER_INCLUDE_DIR 	- the 3dpointer include directories
# 3DPOINTER_LIBRARIES 		- the libraries needed to use the 3Dpointer

find_path(3DPOINTER_INCLUDE_DIR StereoCameraAccess.h)

find_library(3DPOINTER_LIBRARY NAMES StereoCameraComponent)

set(3DPOINTER_LIBRARIES ${3DPOINTER_LIBRARY})
set(3DPOINTER_INCLUDE_DIRS ${3DPOINTER_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set 3DPOINTER_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(3Dpointer DEFAULT_MSG
									3DPOINTER_LIBRARY 3DPOINTER_INCLUDE_DIR)
mark_as_advanced(3DPOINTER_INCLUDE_DIR 3DPOINTER_LIBRARY)

