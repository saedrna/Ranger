# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#.rst:
# FindMosek
# --------
#
# Find the Mosek library (libMosek).
#
# Imported targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the following :prop_tgt:`IMPORTED` targets:
#
# ``Mosek::Mosek``
#   The Mosek library, if found.
#
# Result variables
# ^^^^^^^^^^^^^^^^
#
# This module will set the following variables in your project:
#
# ``Mosek_FOUND``
#   true if the Mosek headers and libraries were found
# ``Mosek_INCLUDE_DIR``
#   the directory containing the Mosek headers
# ``Mosek_INCLUDE_DIRS``
#   the directory containing the Mosek headers
# ``Mosek_LIBRARIES``
#   Mosek libraries to be linked
#
# Cache variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may also be set:
#
# ``Mosek_INCLUDE_DIR``
#   the directory containing the Mosek headers
# ``Mosek_LIBRARY``
#   the path to the Mosek library

find_path(Mosek_INCLUDE_DIR "mosek.h")

set(Mosek_NAMES ${Mosek_NAMES} "mosek" "mosek64" "mosek64_9_1")
foreach(name ${Mosek_NAMES})
  list(APPEND Mosek_NAMES_DEBUG "${name}" "${name}d")
endforeach()

if(NOT Mosek_LIBRARY)
  find_library(Mosek_LIBRARY_RELEASE NAMES ${Mosek_NAMES})
  find_library(Mosek_LIBRARY_DEBUG NAMES ${Mosek_NAMES_DEBUG})
  include(SelectLibraryConfigurations)
  select_library_configurations(Mosek)
  mark_as_advanced(Mosek_LIBRARY_RELEASE Mosek_LIBRARY_DEBUG)
endif()
unset(Mosek_NAMES)
unset(Mosek_NAMES_DEBUG)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Mosek
                                  REQUIRED_VARS Mosek_LIBRARY Mosek_INCLUDE_DIR
                                  VERSION_VAR Mosek_VERSION_STRING)

if(Mosek_FOUND)
  set(Mosek_LIBRARIES ${Mosek_LIBRARY})
  set(Mosek_INCLUDE_DIRS "${Mosek_INCLUDE_DIR}")

  if(NOT TARGET Mosek::Mosek)
    add_library(Mosek::Mosek UNKNOWN IMPORTED)
    if(Mosek_INCLUDE_DIRS)
      set_target_properties(Mosek::Mosek PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${Mosek_INCLUDE_DIRS}")
    endif()
    if(EXISTS "${Mosek_LIBRARY}")
      set_target_properties(Mosek::Mosek PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "C"
        IMPORTED_LOCATION "${Mosek_LIBRARY}")
    endif()
    if(EXISTS "${Mosek_LIBRARY_RELEASE}")
      set_property(TARGET Mosek::Mosek APPEND PROPERTY
        IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(Mosek::Mosek PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
        IMPORTED_LOCATION_RELEASE "${Mosek_LIBRARY_RELEASE}")
    endif()
    if(EXISTS "${Mosek_LIBRARY_DEBUG}")
      set_property(TARGET Mosek::Mosek APPEND PROPERTY
        IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(Mosek::Mosek PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "C"
        IMPORTED_LOCATION_DEBUG "${Mosek_LIBRARY_DEBUG}")
    endif()
  endif()
endif()

mark_as_advanced(Mosek_INCLUDE_DIR Mosek_LIBRARY)
