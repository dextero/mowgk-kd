set(SANDBOX_FOUND FALSE)
set(SANDBOX_INCLUDE_DIR NOTFOUND)
set(SANDBOX_LIBRARIES NOTFOUND)

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/sandbox/lib/libsandbox.a)
    set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/sandbox/cmake ${CMAKE_MODULE_PATH})

    find_package(AssImp REQUIRED)
    find_package(OpenGL REQUIRED)
    find_package(DevIL REQUIRED)
    find_package(GLEW REQUIRED)

    set(SANDBOX_FOUND TRUE)
    set(SANDBOX_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/sandbox/include
                            ${ASSIMP_INCLUDE_DIR}
                            ${OPENGL_INCLUDE_DIR}
                            ${IL_INCLUDE_DIR}
                            ${GLEW_INCLUDE_DIR})
    set(SANDBOX_LIBRARIES ${CMAKE_CURRENT_LIST_DIR}/sandbox/lib/libsandbox.a
                          ${ASSIMP_LIBRARIES}
                          ${OPENGL_LIBRARIES}
                          ${IL_LIBRARIES} ${ILU_LIBRARIES}
                          ${GLEW_LIBRARY})
endif()
