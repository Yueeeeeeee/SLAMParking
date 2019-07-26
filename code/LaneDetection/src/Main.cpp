/*
 * Main.c
 *
 *  Created on: Jan 14, 2018
 *      Author: martin, christian
 */

#include "LaneDetection.h"
#include "VideoProviders/CameraVideoProvider.h"
#include "VideoProviders/FileVideoProvider.h"
#include "VideoProviders/VideoProvider.h"
#include "VideoConsumers/VideoConsumer.h"
#include "VideoConsumers/VideoDisplay.h"
#include "VideoConsumers/VideoFileWriter.h"
#include "VideoConsumers/VideoStreamer.h"
#include "VideoConsumers/VideoSocket.h"
#include "ResultConsumers/ResultConsumer.h"
#include "ResultConsumers/ResultServer.h"
#include "Frame_ts.h"
#include "PrecedingRoverDetection.h"
#include "PrecedingRover_ts.h"
#include "ClientServer/Data.h"
#include <signal.h>
#ifdef MEASSURE_EXECUTION_TIME
#include <chrono>  // for high_resolution_clock
#endif

// Visualization options
const int DONT_SHOW = 0; // Don't show the results
const int SHOW_STREAM = 1; // Show results as a stream
const int SAVE_VIDEO = 2; // Save the results as a video
const int PROVIDE_STREAM = 3;
const int SAVE_INPUT_VIDEO = 4; //not really a show option
const int PROVIDE_SOCKET = 5;

VideoProvider* videoProvider(nullptr);
LaneDetection* laneDetection(nullptr);
VideoConsumer* videoConsumer(nullptr);
ResultConsumer* resultConsumer(nullptr);
Frame_ts currentFrame;
PrecedingRover_ts precedingRover;

static bool terminationRequest = false;
static int showOption = 0;

char* getCmdOption(char ** begin, char ** end, const std::string & option) {
	char ** itr = std::find(begin, end, option);
	if (itr != end && ++itr != end) {
		return *itr;
	}
	return 0;
}

bool cmdOptionExists(char** begin, char** end, const std::string& option) {
	return std::find(begin, end, option) != end;
}

void sigHandler(int) {
	std::cout << "Ctr+C received\n";
	terminationRequest = true;
}

void processingLoop() {

	//stuff for measuring Time
	int totalFrames = 0;
	const int64 start = cv::getTickCount();
	int counter = 0;
	int64 alsoStart = start;

	while (!terminationRequest) {
#ifdef MEASSURE_EXECUTION_TIME
		std::cout << "============ new Frame ============" << std::endl;
		auto startReadFrame = std::chrono::high_resolution_clock::now();
#endif
		cv::Mat frame = videoProvider->getNextFrame();
#ifdef MEASSURE_EXECUTION_TIME
		auto finishReadFrame = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsedReadFrame = finishReadFrame - startReadFrame;
		std::cout << "Read Frame:\t\t" << elapsedReadFrame.count() * 1000 << " ms" << std::endl;
#endif

		currentFrame.setFrame(frame);
		if (currentFrame.getFrame().empty()) {
			std::cout << "Video over.\n";
			break;
		}

#ifdef MEASSURE_EXECUTION_TIME
		std::cout << "Image Processing:" << std::endl;
		auto startProcessImage = std::chrono::high_resolution_clock::now();
#endif
		laneDetection->processImage(currentFrame.getFrame());
#ifdef MEASSURE_EXECUTION_TIME
		auto finishProcessImage = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsedProcessImage = finishProcessImage - startProcessImage;
		std::cout << "- total:\t\t" << elapsedProcessImage.count() * 1000 << " ms" << std::endl;
#endif

		if (videoConsumer != nullptr) {
			if (showOption != SAVE_INPUT_VIDEO) {
				videoConsumer->consumeFrame(laneDetection->getResultFrame());
			} else {
				videoConsumer->consumeFrame(laneDetection->getBaseFrame());
			}
		}

		LDValues currentValues = laneDetection->getResultData();
		// These values need do be put in LDValues
		currentValues.precedingRoverDistance = precedingRover.getDistance();
		currentValues.precedingRoverId = precedingRover.getId();
		currentValues.precedingRoverRecognized =
				precedingRover.isRoverRecognized();

		resultConsumer->consumeResultData(currentValues);

		counter++;
		if (counter % 30 == 0) {
			counter = 0;
			double timeDiff = (cv::getTickCount() - alsoStart)
					/ cv::getTickFrequency();
			alsoStart = cv::getTickCount();
			std::cout << "\t\t\t\t\t\t\tRunning at: " << 30 / timeDiff
					<< " fps\n";
		}

		totalFrames++;
	}

	double totalTime = (cv::getTickCount() - start) / cv::getTickFrequency();
	std::cout << "Total time: " << totalTime << "s\nTotal frames: "
			<< totalFrames << "\n";
	std::cout << "Average time per frame: " << 1000 * totalTime / totalFrames
			<< "ms\n";
	std::cout << "Average Framerate: " << totalFrames / totalTime << " fps\n";

}

int main(int argc, char** argv) {
//int showOption = 0;
	if (argc == 1) {
		videoProvider = new FileVideoProvider("test.mp4");
		showOption = 1;
		std::cout
				<< "No commandlineparameters. Will try to use Video test.mp4 and Display the result on the Screen\n";
		std::cout << "Usage: [-c [cameraId] | [-v fileName]] -o showOption\n";
	} else {
		if (!cmdOptionExists(argv, argv + argc, "-v")
				&& !cmdOptionExists(argv, argv + argc, "-c")) {
			std::cout << "You have to provide Camera or Video-file.\n";
			std::cout
					<< "Usage: [-c [cameraId] | [-v fileName]] -o showOption\n";
		}
		if (!cmdOptionExists(argv, argv + argc, "-o")) {
			std::cout << "Missing showOption, so nothing will be shown\n";
		} else {
			showOption = (int) (getCmdOption(argv, argv + argc, "-o")[0] - 48);
			if (cmdOptionExists(argv, argv + argc, "-c")) {
				videoProvider = new CameraVideoProvider(
						(int) (getCmdOption(argv, argv + argc, "-c")[0] - 48));

			} else if (cmdOptionExists(argv, argv + argc, "-v")) {
				videoProvider = new FileVideoProvider(
						getCmdOption(argv, argv + argc, "-v"));

			} else {
				std::cout << "No source specified\n";
			}

		}
	}

	if (showOption == DONT_SHOW) {
		//currently nothing
	} else if (showOption == SHOW_STREAM) {
		videoConsumer = new VideoDisplay("Result with lanes and distances",
				1000, 700, true);
	} else if (showOption == SAVE_VIDEO || showOption == SAVE_INPUT_VIDEO) {
		videoConsumer = new VideoFileWriter("ResultVideo.avi",
				videoProvider->getWidth(), videoProvider->getHeight());
	} else if (showOption == PROVIDE_STREAM) {
		videoConsumer = new VideoStreamer();
	} else if (showOption == PROVIDE_SOCKET) {
		videoConsumer = new VideoSocket();
	}

	laneDetection = new LaneDetection((showOption > 0),
			videoProvider->getWidth(), videoProvider->getHeight(),
			precedingRover);

	resultConsumer = new ResultServer();

	if (!videoProvider->isReady()) {
		std::cout << "Couldn't open VideoReader. Terminating\n";
		exit(-1);
	}

//this is used for gracefull shutdown when ctrl+c is pressed
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = sigHandler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	PrecedingRoverDetection precedingRoverDetection(currentFrame,
			&precedingRover);
	std::thread thread_precedingRoverDetection(&PrecedingRoverDetection::Run,
			&precedingRoverDetection);
	thread_precedingRoverDetection.detach();

//now doing computervision
	processingLoop();

	delete laneDetection;
	delete videoProvider;
	delete videoConsumer;
	delete resultConsumer;

	return 0;
}

