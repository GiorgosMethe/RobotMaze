# The set of languages for which implicit dependencies are needed:
set(CMAKE_DEPENDS_LANGUAGES
  )
# The set of files for implicit dependencies of each language:

# Preprocessor definitions for this target.
set(CMAKE_TARGET_DEFINITIONS
  "BOOST_ALL_NO_LIB"
  "BUILD_NEAT_DLL"
  "NOMINMAX"
  "TIXML_USE_STL"
  )

# Targets to which this target links.
set(CMAKE_TARGET_LINKED_INFO_FILES
  )

# The include file search paths:
set(CMAKE_C_TARGET_INCLUDE_PATH
  "../NEAT/include"
  "../NEAT/../../../tinyxmldll/include"
  "../NEAT/../../../../Libraries/boost-trunk"
  "../NEAT/../../../JGTL/include"
  "../NEAT/../../../zlib"
  "../NEAT/../../../Board/include"
  )
set(CMAKE_CXX_TARGET_INCLUDE_PATH ${CMAKE_C_TARGET_INCLUDE_PATH})
set(CMAKE_Fortran_TARGET_INCLUDE_PATH ${CMAKE_C_TARGET_INCLUDE_PATH})
set(CMAKE_ASM_TARGET_INCLUDE_PATH ${CMAKE_C_TARGET_INCLUDE_PATH})
