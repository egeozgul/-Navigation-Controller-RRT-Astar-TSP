find_path(VRPN_INCLUDE_DIR vrpn_Tracker.h PATHS /usr/local/include)
find_library(VRPN_LIBRARY vrpn PATHS /usr/local/lib)
find_library(QUAT_LIBRARY quat PATHS /usr/local/lib)

set(VRPN_LIBRARIES ${VRPN_LIBRARY} ${QUAT_LIBRARY})
set(VRPN_INCLUDE_DIRS ${VRPN_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VRPN DEFAULT_MSG VRPN_LIBRARY VRPN_INCLUDE_DIR)
