# - Try to find Sphinxbase
# Once done this will define
#
# Sphinxbase_FOUND          - set to true if Sphinxbase was found
# Sphinxbase_LIBRARIES      - link these to use Sphinxbase
# Sphinxbase_INCLUDE_DIR    - path to Sphinxbase header files

if (Sphinxbase_LIBRARIES AND Sphinxbase_INCLUDE_DIR)
  # in cache already
  set(Sphinxbase_FOUND TRUE)
else (Sphinxbase_LIBRARIES AND Sphinxbase_INCLUDE_DIR)

  set(CANDIDATE_LIB_DIR
    /usr/lib
    /usr/local/lib
    /usr/lib/sphinxbase
    /usr/local/lib/sphinxbase
    /usr/local/sphinxbase/lib
    /opt/sphinxbase/lib
  )
  
  set(CANDIDATE_INC_DIR
    /usr/include
    /usr/include/sphinxbase
    /usr/local/include
    /usr/local/include/sphinxbase
    /usr/local/sphinxbase/include
    /opt/sphinxbase/include
  )

  find_path(Sphinxbase_INCLUDE_DIR sphinx_config.h ${CANDIDATE_INC_DIR})
  find_library(Sphinxbase_LIBRARIES sphinxbase ${CANDIDATE_LIB_DIR})

  # status output
  include(FindPackageHandleStandardArgs)

  find_package_handle_standard_args(Sphinxbase
    DEFAULT_MSG
    Sphinxbase_INCLUDE_DIR
    Sphinxbase_LIBRARIES
  )
  mark_as_advanced(
    Sphinxbase_INCLUDE_DIR
    Sphinxbase_LIBRARIES
  )

endif (Sphinxbase_LIBRARIES AND Sphinxbase_INCLUDE_DIR)