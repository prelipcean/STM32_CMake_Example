# ===================================================================
# Platform helpers
# ===================================================================
#
# platform_add_module(<name>
#                     [SOURCES <src>...]           # none -> header only module
#                     [INCLUDES <dir>...]          # public include dirs
#                     [DEPENDS <module>...]        # used by the public headers
#                     [PRIVATE_DEPENDS <module>...]) # used only by the sources
#
# Every module is an OBJECT library (all objects are linked, so strong ISR
# definitions always replace the weak ones from the startup file; unused code
# is removed by --gc-sections). A module only sees the headers of the modules
# it depends on, which keeps the layers (App -> VFB -> BSP -> MCAL) honest.
# Modules are registered and linked into the firmware automatically.
# ===================================================================

function(platform_add_module NAME)
  cmake_parse_arguments(M "" "" "SOURCES;INCLUDES;DEPENDS;PRIVATE_DEPENDS"
                        ${ARGN})

  if(M_SOURCES)
    add_library(${NAME} OBJECT ${M_SOURCES})
    target_include_directories(${NAME} PUBLIC ${M_INCLUDES})
    target_link_libraries(${NAME} PUBLIC ${M_DEPENDS}
                          PRIVATE platform_options ${M_PRIVATE_DEPENDS})
  else()
    add_library(${NAME} INTERFACE)
    target_include_directories(${NAME} INTERFACE ${M_INCLUDES})
    target_link_libraries(${NAME} INTERFACE ${M_DEPENDS})
  endif()

  set_property(GLOBAL APPEND PROPERTY PLATFORM_MODULES ${NAME})
endfunction()

# Link every registered module into <target>
function(platform_link_modules TARGET)
  get_property(_modules GLOBAL PROPERTY PLATFORM_MODULES)
  target_link_libraries(${TARGET} PRIVATE ${_modules})
endfunction()
