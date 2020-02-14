#include <iostream>
#include <fstream>

#define NCH (1024*512)
#define CHANNELS_PER_WORKER (1024*512)

// Old header for JF4M data
struct JF_rawheader {
    uint64_t framenumber;
    uint32_t subframenumber;
    uint32_t packetnumber;
    uint64_t bunchid;
    uint64_t timestamp;
    uint16_t moduleid;
    uint16_t xcoord;
    uint16_t ycoord;
    uint16_t zcoord;
    uint32_t debug;
    uint16_t rrnumber;
    uint8_t detectortype;
    uint8_t headerversion;
};

class JungfrauFile {
    std::ifstream filebin; // file descriptor
public:
    JungfrauFile(char *fformat, int fileindex);
    ~JungfrauFile();
    bool ReadHeader(JF_rawheader *header);
    bool IgnoreHeader();
    bool IgnoreFrames(uint32_t nframes);
    bool IgnoreFramesFromBeg(uint32_t nframes);
    bool ReadFrame(uint16_t *frame, uint32_t store, uint32_t ignore_in_front, uint32_t ignore_afterwards);
    bool ReadFrame(uint16_t *frame, uint32_t start_channel);
};

struct JF_frame {
    uint16_t imagedata[NCH];
};
