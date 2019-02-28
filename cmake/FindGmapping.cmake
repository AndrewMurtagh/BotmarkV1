# - Try to find Gmapping
# Once done this will define
#
# Gmapping_FOUND          - set to true if Gmapping was found
# Gmapping_INCLUDE_DIR    - path to Gmapping header files
# Gmapping_LIBRARIES      - link these to use Gmapping

if (Gmapping_INCLUDE_DIR AND Gmapping_LIBRARIES)
  # in cache already
  set(Gmapping_FOUND TRUE)
else (Gmapping_INCLUDE_DIR AND Gmapping_LIBRARIES)

  set(CANDIDATE_INC_DIR
    /usr/include
    /usr/include/gmapping
    /usr/local/include
    /usr/local/include/gmapping
    /usr/local/gmapping/include
    /opt/gmapping/include
  )


  set(CANDIDATE_LIB_DIR
    /usr/lib
    /usr/local/lib
    /usr/lib/gmapping
    /usr/local/lib/gmapping
    /usr/local/gmapping/lib
    /opt/gmapping/lib
  )

  find_path(Gmapping_INCLUDE_DIR gridfastslam/gridslamprocessor.h ${CANDIDATE_INC_DIR})




  find_library(CONFIG_LIBRARY configfile ${CANDIDATE_LIB_DIR})
  find_library(GFS_LIBRARY gridfastslam ${CANDIDATE_LIB_DIR})
  #find_library(GUI_LIBRARY gui ${CANDIDATE_LIB_DIR})
  find_library(LOG_LIBRARY log ${CANDIDATE_LIB_DIR})
  find_library(SCANMATCHER_LIBRARY scanmatcher ${CANDIDATE_LIB_DIR})
  find_library(SENSOR_BASE_LIBRARY sensor_base ${CANDIDATE_LIB_DIR})
  find_library(SENSOR_ODO_LIBRARY sensor_odometry ${CANDIDATE_LIB_DIR})
  find_library(SENSOR_RANGE_LIBRARY sensor_range ${CANDIDATE_LIB_DIR})
  find_library(SENSOR_UTILS_LIBRARY utils ${CANDIDATE_LIB_DIR})
  set(Gmapping_LIBRARIES
    ${CONFIG_LIBRARY}
    ${GFS_LIBRARY}
#   ${GUI_LIBRARY}
    ${LOG_LIBRARY}
    ${SCANMATCHER_LIBRARY}
    ${SENSOR_BASE_LIBRARY}
    ${SENSOR_ODO_LIBRARY}
    ${SENSOR_RANGE_LIBRARY}
    ${SENSOR_UTILS_LIBRARY}
  )

  # status output
  include(FindPackageHandleStandardArgs)

  find_package_handle_standard_args(Gmapping
    DEFAULT_MSG
    Gmapping_INCLUDE_DIR
    Gmapping_LIBRARIES
  )
  mark_as_advanced(
    Gmapping_INCLUDE_DIR
    Gmapping_LIBRARIES
  )

endif (Gmapping_INCLUDE_DIR AND Gmapping_LIBRARIES)