import cv2

def check_opencv_gstreamer_support():
    # 获取OpenCV版本
    version = cv2.__version__
    print(f"OpenCV版本: {version}")

    # 检查是否支持GStreamer
    print(cv2.getBuildInformation())
    # gstreamer_supported = cv2.getBuildInformation().find("GStreamer") != "NO"
    # if gstreamer_supported:
    #     print("OpenCV支持GStreamer")
    # else:
    #     print("OpenCV不支持GStreamer")

check_opencv_gstreamer_support()
