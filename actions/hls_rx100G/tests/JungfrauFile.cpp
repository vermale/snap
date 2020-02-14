#include <cstdio>
#include <fstream>

#include "JungfrauFile.h"

JungfrauFile::JungfrauFile(char *fformat, int fileindex) { // constructor
    char fname[1000];
    sprintf(fname,fformat,fileindex);
    filebin.open((const char *)(fname), filebin.in | filebin.binary);
}

JungfrauFile::~JungfrauFile() {
    filebin.close();

}

// only reads necessary number of channels
// also helps for vectorization as frame can be externally aligned

bool JungfrauFile::ReadHeader(JF_rawheader *header) {
      filebin.read((char *)header, sizeof(JF_rawheader));
      if (filebin.good()) return true;
      return false;
}

bool JungfrauFile::IgnoreHeader() {
      filebin.seekg(sizeof(JF_rawheader),filebin.cur);
      if (filebin.good()) return true;
      return false;
}

bool JungfrauFile::IgnoreFrames(uint32_t nframes) {
      filebin.seekg((sizeof(JF_frame)+sizeof(JF_rawheader))*nframes,filebin.cur);
      if (filebin.good()) return true;
      return false;
}

bool JungfrauFile::IgnoreFramesFromBeg(uint32_t nframes) {
      filebin.seekg((sizeof(JF_frame)+sizeof(JF_rawheader))*nframes,filebin.beg);
      if (filebin.good()) return true;
      return false;
}

bool JungfrauFile::ReadFrame(uint16_t *frame, uint32_t store, uint32_t ignore_in_front, uint32_t ignore_afterwards) {

      filebin.seekg(sizeof(JF_rawheader)+ignore_in_front*sizeof(uint16_t),filebin.cur);
      filebin.read((char *)frame,store*sizeof(uint16_t));
      filebin.seekg(ignore_afterwards*sizeof(uint16_t),filebin.cur);

      if (filebin.good()) return true;
      return false;
}

bool JungfrauFile::ReadFrame(uint16_t *frame, uint32_t start_channel) {
      return ReadFrame(frame, CHANNELS_PER_WORKER, start_channel, NCH - start_channel - CHANNELS_PER_WORKER);
}
