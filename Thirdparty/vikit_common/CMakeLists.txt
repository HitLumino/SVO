LIST(APPEND SOURCEFILES src/atan_camera.cpp
						src/omni_camera.cpp
						src/math_utils.cpp
						src/vision.cpp
						src/performance_monitor.cpp
						src/robust_cost.cpp
						src/user_input_thread.cpp
						src/pinhole_camera.cpp
						src/homography.cpp
						src/img_align.cpp)
						
ADD_LIBRARY(vikit_common SHARED ${SOURCEFILES})

TARGET_LINK_LIBRARIES(vikit_common
			${OpenCV_LIBS}
			${Sophus_LIBRARIES}
)

INSTALL(TARGETS vikit_common
	DESTINATION CMAKE_INSTALL_PREFIX/lib)