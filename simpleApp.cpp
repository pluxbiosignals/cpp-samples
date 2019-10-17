#include "simpleDev.h"

#include <string> //// If you want to define the mac address in a variable

int main() {
	try
	{
		///// Use this block to search for PLUXs devices (via BTH or USB)
		SimpleDev::VDevInfo devs = SimpleDev::find();
		for (int i = 0; i < devs.size(); i++) {
			printf("%s - %s\n", devs[i].path.c_str(), devs[i].description.c_str());
		}

		///// Use this block to connect to a device

		////const std::string MAC_Addres = "BTHxx:xx:xx:xx:xx:xx";   /// Devices mac address
		const std::string MAC_Addres = "USB0";										/// Use the UBS adapter (Only for windows)

		SimpleDev dev(MAC_Addres);
		puts("Connected to device. Press Enter to exit.");
		std::string ver = dev.version();    // get device version string
		printf("Device version: %s\n", ver.c_str());

		dev.start(1000, { 1, 2, 3, 4, 5, 6, 7, 8 });										/// Start the device with sampling rate = 1000, all the channels and at 16bit resolution

		dev.trigger(false);

		SimpleDev::VFrame frames(100); // initialize the frames vector with 100 frames (From simpleDev cpp)
		int nSamples = 0;
		int maxSamples = 4000;
		do
		{
			dev.read(frames); // get 100 frames from device
			const SimpleDev::Frame &f = frames[0];  // get a reference to the first frame of each 100 frames block
			printf("%d : %d ; %d %d %d %d %d %d %d %d\n",   // dump the first frame
				f.seq,
				f.digital[0],
				f.analog[0], f.analog[1], f.analog[2], f.analog[3],
				f.analog[4], f.analog[5], f.analog[6], f.analog[7]);
			nSamples += 100;
			} while (nSamples <= maxSamples); // until a key is pressed
		dev.stop(); // stop acquisition
	} // dev is destroyed here (it goes out of scope)
	catch (Plux::Exception &e)
	{
	printf("Exception: %s\n", e.getDescription());
	}
	return 0;
}