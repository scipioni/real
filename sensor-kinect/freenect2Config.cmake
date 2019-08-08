FIND_LIBRARY(freenect2_LIBRARY freenect2
    PATHS /usr/local/lib
    NO_DEFAULT_PATH
)
SET(freenect2_LIBRARIES ${freenect2_LIBRARY} )
FIND_PATH(freenect2_INCLUDE_DIR libfreenect2/libfreenect2.hpp
    PATHS /usr/local/include
    NO_DEFAULT_PATH
)

IF("")
    SET(freenect2_INCLUDE_DIRS ${freenect2_INCLUDE_DIR} ${freenect2_INCLUDE_DIR}/tinythread)
ENDIF("")
