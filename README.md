# CloudMovement-CPP
PhD project for detecting cloud movement using Optical Flow and forecasting advection using Boids Algorithm.
This is the variant written in C++.

## OpenCV Library
Please get the relevant OpenCV library for your operating system. In Releases you can find OpenCV 4.5.0. source. Download and build from source using these steps.

## Running Code
You should add the following settings to your local project:

**Header Search Path:** /path/to/opencv/folder/4.5.0_3/include/opencv4

**Library Search Path:** /path/to/opencv/folder/4.5.0_3/lib

**Other Linker Flags:** -I/usr/local/Cellar/opencv/4.5.0_3/include/opencv4 -L/usr/local/Cellar/opencv/4.5.0_3/lib -lopencv_gapi -lopencv_stitching -lopencv_alphamat -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dnn_objdetect -lopencv_dnn_superres -lopencv_dpm -lopencv_highgui -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hfs -lopencv_img_hash -lopencv_intensity_transform -lopencv_line_descriptor -lopencv_mcc -lopencv_quality -lopencv_rapid -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_sfm -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_superres -lopencv_optflow -lopencv_surface_matching -lopencv_tracking -lopencv_datasets -lopencv_text -lopencv_dnn -lopencv_plot -lopencv_videostab -lopencv_videoio -lopencv_viz -lopencv_xfeatures2d -lopencv_shape -lopencv_ml -lopencv_ximgproc -lopencv_video -lopencv_xobjdetect -lopencv_objdetect -lopencv_calib3d -lopencv_imgcodecs -lopencv_features2d -lopencv_flann -lopencv_xphoto -lopencv_photo -lopencv_imgproc -lopencv_core
