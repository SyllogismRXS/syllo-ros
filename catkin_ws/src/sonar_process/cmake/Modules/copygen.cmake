# begin copygen.cmake
function(copygen BUILD_DIR INC_DIR)

  # Copy generated header files to include path
  File(GLOB_RECURSE GEN_INCLUDE_FILES 
    RELATIVE ${BUILD_DIR}/idl
    ${BUILD_DIR}/idl/dds/*.h
    )

  foreach(I ${GEN_INCLUDE_FILES})
    configure_file(${BUILD_DIR}/idl/${I} ${INC_DIR}/${I} @ONLY)
  endforeach()
  
endfunction()

copygen(${BUILD_DIR} ${INC_DIR})
# end copygen.cmake
