/**
 * \file
 * \copyright  Copyright 2015 PLUX - Wireless Biosignals, S.A.
 * \version    1.0
 * \date       March 2015
 * 
 * \section LICENSE
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
 */


#include "simpleDev.h"

using namespace Plux;

/*****************************************************************************/

String SimpleDev::version(void)
{
   char verNumStr[20];

   Properties props = sigDev.getProperties();
   int ver = props["fwVersion"].i;
   sprintf_s(verNumStr, " v%d.%d", (ver>>8) & 0xFF, ver & 0xFF);

   return *props["description"].s + verNumStr;
}

/*****************************************************************************/

void SimpleDev::start(float freq, int portMask, int nBits)
{
   sigDev.start(freq, portMask, nBits);

   sigDev.nChannels = 0;
   for(int i = 0; i < 32; i++)
   {
      if (portMask & 1)   sigDev.nChannels++;
      portMask >>= 1;
   }

   if (sigDev.nChannels > 8)  sigDev.nChannels = 8;
}

/*****************************************************************************/

void SimpleDev::start(float freq, const VInt &ports, int nBits)
{
   if (ports.empty())
   {
      sigDev.start(freq, 0xFF, nBits);
      sigDev.nChannels = 8;
   }
   else
   {
      sigDev.start(freq, ports, nBits);
      sigDev.nChannels = (ports.size() > 8) ? 8 : ports.size();
   }

}

/*****************************************************************************/

void SimpleDev::read(VFrame &frames)
{
   if (sigDev.nChannels == 0)     throw Error::InvalidOperation("N channels");

   if (frames.empty())   frames.resize(100);

   sigDev.frame = frames.begin();
   sigDev.end = frames.end();

   sigDev.loop();
}

/*****************************************************************************/

void SimpleDev::stop(void)
{
   sigDev.stop();

   sigDev.nChannels = 0;
}

/*****************************************************************************/

void SimpleDev::trigger(const VBool &digitalOutput)
{
   if (digitalOutput.empty())
      trigger(false);
   else
      trigger(digitalOutput[0]);
}

/*****************************************************************************/

bool SimpleDev::SignalsDevX::onEvent(const Event &evt)
{
   if (evt.type == Event::TypeDigInUpdate)
      lastDigVal = static_cast<const EvtDigInUpdate&>(evt).state;

   return false;
}

/*****************************************************************************/

bool SimpleDev::SignalsDevX::onRawFrame(int nSeq, const int data[])
{
   frame->seq = nSeq;
   memcpy(frame->analog, data, nChannels*sizeof(int));
   frame->digital[0] = lastDigVal;

   frame++;
   return (frame == end);
}

/*****************************************************************************/
