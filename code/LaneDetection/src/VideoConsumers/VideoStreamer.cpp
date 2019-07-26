/*
 * VideoStreamer.cpp
 *
 *  Created on: May 15, 2018
 *      Author: christian
 */

#include "VideoStreamer.h"

using namespace Pistache;
using namespace cv;
using namespace std;

const static int IMAGES_MAX = 3;
static int imageCurrent = 0;
static String imageFolder = "";

struct ImageRequestHandler: public Http::Handler {
HTTP_PROTOTYPE(ImageRequestHandler)

	void onRequest(const Http::Request& request, Http::ResponseWriter writer) {
		(void) request; //suppress compiler warning
		int serve = imageCurrent - 1;
		if (serve < 0) {
			serve = IMAGES_MAX - 1;
		}
		Http::serveFile(writer,
				(imageFolder + "/"+ std::to_string(imageCurrent) + ".jpg").c_str());
	}
};

// Function listening on port 9080
static void listenerFunction() {
	Pistache::Address addr(Pistache::Ipv4::any(), Pistache::Port(9080));
	auto opts = Pistache::Http::Endpoint::options().threads(1).flags(
			Tcp::Options::ReuseAddr);

	Http::Endpoint server(addr);
	server.init(opts);
	server.setHandler(Http::make_handler<ImageRequestHandler>());
	server.serve();

	// not sure if this will ever be called
	server.shutdown();
}

VideoStreamer::VideoStreamer() {
	imageFolder="./";
	struct stat st;
	if (stat("/ramFolder", &st) == 0) {
		if ((st.st_mode & S_IFDIR) != 0) {
			imageFolder="/ramFolder";
		}
	}
	current=0;
	cout << "Folder for temp images will be: " << imageFolder <<endl;
	std::thread imageServer(listenerFunction);
	imageServer.detach();

}

VideoStreamer::~VideoStreamer() {
}

void VideoStreamer::consumeFrame(cv::Mat frame) {
	cv::Size size(424, 240);
	Mat rescaledFrame;
	resize(frame, rescaledFrame, size);

	std::vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(20); // (0-100)

	imwrite((imageFolder +"/"+ std::to_string(current) + ".jpg").c_str(),
			rescaledFrame, compression_params);
	current++;
	if (current >= IMAGES_MAX) {
		current = 0;
	}
	imageCurrent = current;

}
