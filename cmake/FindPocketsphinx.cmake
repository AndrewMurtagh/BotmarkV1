# - Try to find Pocketsphinx
# Once done this will define
#
# Pocketsphinx_FOUND          - set to true if Pocketsphinx was found
# Pocketsphinx_LIBRARIES      - link these to use Pocketsphinx
# Pocketsphinx_INCLUDE_DIR    - path to Pocketsphinx header files

if (Pocketsphinx_LIBRARIES AND Pocketsphinx_INCLUDE_DIR)
  # in cache already
  set(Pocketsphinx_FOUND TRUE)
else (Pocketsphinx_LIBRARIES AND Pocketsphinx_INCLUDE_DIR)

  set(CANDIDATE_LIB_DIR
    /usr/lib
    /usr/local/lib
    /usr/lib/pocketsphinx
    /usr/local/lib/pocketsphinx
    /usr/local/pocketsphinx/lib
    /opt/pocketsphinx/lib
  )
  
  set(CANDIDATE_INC_DIR
    /usr/include
    /usr/include/pocketsphinx
    /usr/local/include
    /usr/local/include/pocketsphinx
    /usr/local/pocketsphinx/include
    /opt/pocketsphinx/include
  )

  find_path(Pocketsphinx_INCLUDE_DIR 
    NAMES pocketsphinx.h 
    PATHS ${CANDIDATE_INC_DIR})



  find_library(Pocketsphinx_LIBRARIES pocketsphinx ${CANDIDATE_LIB_DIR})

  # status output
  include(FindPackageHandleStandardArgs)

  find_package_handle_standard_args(Pocketsphinx
    DEFAULT_MSG
    Pocketsphinx_INCLUDE_DIR
    Pocketsphinx_LIBRARIES
  )
  mark_as_advanced(
    Pocketsphinx_INCLUDE_DIR
    Pocketsphinx_LIBRARIES
  )

endif (Pocketsphinx_LIBRARIES AND Pocketsphinx_INCLUDE_DIR)