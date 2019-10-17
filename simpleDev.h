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

#ifndef _SIMPLEDEVHEADER_
#define _SIMPLEDEVHEADER_

#include "plux.h"


class SimpleDev
{
public:
// Type definitions

   struct Frame
   {
      int   seq;
      bool  digital[4];
      int   analog[8];
   };
   typedef std::vector<Frame> VFrame;

   // Type equivalencies for BITalino application source code compatibility
   typedef Plux::String    String;
   typedef Plux::Bools     VBool;
   typedef Plux::Ints      VInt;
   typedef Plux::DevInfos  VDevInfo;

// Static methods

   static VDevInfo find(void) { return Plux::BaseDev::findDevices(); }

// Instance methods

   SimpleDev(const String &path) : sigDev(path) {}

   String version(void);

   void start(float freq, int portMask, int nBits = 16);
   void start(float freq, const VInt &ports = VInt(), int nBits = 16);

   void stop(void);

   void read(VFrame &frames);

   void trigger(bool digitalOutput) { sigDev.setDOut(digitalOutput); }
   void trigger(const VBool &digitalOutput = VBool());

private:
   class SignalsDevX : public Plux::SignalsDev
   {
   public:
      SignalsDevX(const String &path) : SignalsDev(path), nChannels(0) {}

      VFrame::iterator  frame, end;
      int               nChannels;

   private:
      bool onEvent(const Plux::Event &evt);
      bool onRawFrame(int nSeq, const int data[]);

      bool lastDigVal;
   } sigDev;
};

#endif // _SIMPLEDEVHEADER_
