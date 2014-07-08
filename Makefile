all : link

link:compile
	g++ src/uvc_cam_node.o -l opencv_core -l opencv_imgproc -l opencv_objdetect -l opencv_highgui -L -L/opt/ros/groovy/lib /opt/ros/groovy/lib/librosconsole.so -o main 

compile : 
	g++ -Wall -g  -pg -std=c++0x -pthread -I include/ -I /home/maxired/catkin_ws/src/hyve-ros/facetracking/cfg/cpp -c  src/uvc_cam_node.cpp -o src/uvc_cam_node.o

clean : 
	rm main.o
	rm main
