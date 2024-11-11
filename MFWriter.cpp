#define NOMINMAX
#include <iostream>
#include <string>
//#include <boost/noncopyable.hpp>
#include <memory>
#include <atlbase.h> // CComPtr
#include <comdef.h>   // _com_error
#include <DShow.h>    // ICodecAPI
#include <KS.h>       // required by Codecapi.h
#include <Codecapi.h> // Code Properties
#include <cmath>
#include <cstdint>
#include<fstream>

#include <windows.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mferror.h>
#include <mfreadwrite.h>
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid")


//#include <windows.h>
//#include <mfapi.h>
//#include <mfplay.h>
//#include <mfreadwrite.h>
//#include <mferror.h>
//
//#pragma comment(lib, "mf")
//#pragma comment(lib, "mfplat")
//#pragma comment(lib, "mfplay")
//#pragma comment(lib, "mfreadwrite")
//#pragma comment(lib, "mfuuid")



#define MFWRITER_RETURN_ON_ERROR(A) if( !(A) ) { return false; }

// This macro is used when we need the HRESULT code of an operation 
// to take further action before sending an error event to the channel
#define MFWRITER_RETURN_HRESULT_ON_ERROR(A) if( FAILED(A) ) {return A; }

namespace {
    CComPtr<IMFMediaBuffer> createMediaBuffer(const unsigned char* const input);
    uint32_t computeH264BitRate(const uint32_t quality, uint32_t width, uint32_t height,
        double frameRate, size_t bitsPerPixel);
    const double HUNDRED_NANOSECONDS = 10000000.0;
}


class MFInitializer
{
public:
    MFInitializer()
        : _hrCOM(CoInitializeEx(NULL, COINIT_APARTMENTTHREADED))
    {
        _hrMF = MFStartup(MF_VERSION);
        if (!SUCCEEDED(_hrMF))
        {
            // TODO: Handle error conditions like MF_E_BAD_STARTUP_VERSION and 
            // MF_E_DISABLED_IN_SAFEMODE
            // This exception will be thrown across modules only on Windows platforms
            std::cerr << "Failed to initialize MediaFoundation" << std::endl;
            exit(1);
        }
    }

    ~MFInitializer()
    {
        if (SUCCEEDED(_hrMF))
        {
            MFShutdown();
        }

        if (SUCCEEDED(_hrCOM))
        {
            CoUninitialize();
        }
    }

private:
    HRESULT _hrCOM; ///< result of COM initialization
    HRESULT _hrMF;  ///< result of Media Foundation Initialization

};


class MFWriter {
public:
    MFWriter();
    ~MFWriter();

    //channel::IControl& getChannel() const;
    void init();
    void open();
    //void execute(const char* const command, const device::PropertyList& options);
    void close();
    void term();
    //device::IDataSource* getDataSource() const;
    //device::IDataSink* getDataSink() const;
    //bool isSinkDone() const;
    //device::IDataSinkPolling* getSinkPolling() const;
    //double getSpaceLatency() const;
    //bool isSpaceAvailable() const;
    const unsigned char* readbinfile(std::string frame);
    bool write(const unsigned char* input);
    //void frameWritten(size_t width, size_t height) const;
    HRESULT createSinkWriter(UINT32 width, UINT32 height);
    void destroySinkWriter();
    bool throwIf(HRESULT hr);
    static const char* const FILE_NAME;
    static const char* const FRAME_RATE;
    static const char* const QUALITY;
    static const char* const EXPECTED_FRAME_TYPE;
    static const size_t MAX_QUALITY;
    static const double INVALID_QUALITY;
    static const std::string FRAME_TYPE_FOR_MPEG4;
    //channel::IControl& _channel;
    std::unique_ptr<MFInitializer> _mfInit;
    CComPtr<IMFSinkWriter> _sinkWriter;
    DWORD _videoStreamIndex;
    //fl::ustring _fileName;
    double _frameRate;
    UINT32 _quality;
    bool _fileSpaceAvailable;
    LONGLONG _frameStart;
    UINT64 _frameDuration;

};

MFWriter::MFWriter() :
    _mfInit(),
    _sinkWriter(NULL),
    _videoStreamIndex(0),
    //_fileName(),
    _frameRate(0),
    _quality(0),
    _fileSpaceAvailable(true),
    _frameStart(0),
    _frameDuration(0)
{
}


MFWriter::~MFWriter()
{
}

void MFWriter::init() {
    _mfInit.reset(new MFInitializer());
}


void MFWriter::open()
{
    _frameRate = 30;
    _quality = 75;
    _fileSpaceAvailable = true;
}

void MFWriter::close()
{
    destroySinkWriter();
}

void MFWriter::term()
{
    delete this;
}

bool MFWriter::throwIf(HRESULT hr)
{
    if (SUCCEEDED(hr))
    {
        return true;
    }

    switch (hr)
    {
        // All of the following errors are unexpected and fall through to the default case
        // Errors are listed here for debugging purposes, as these are the most common errors 
        // coming from MediaFoundations SinkWriter

        // General init errors
    case MF_E_INVALIDMEDIATYPE:
    {
        //getChannel().sendMessageEvent(new ErrorInfo("mediafoundationplugin:invalidInputs", ""));
        std::cerr << "MF_E_INVALIDMEDIATYPE." << std::endl;
        exit(1);
        return false;
        break;

    }
    case MF_E_TOPO_CODEC_NOT_FOUND:

        // Reading Errors
    case MF_E_INVALIDREQUEST:
    case MF_E_NOTACCEPTING:
    case E_INVALIDARG:

        // File based errors
    case MF_E_INVALID_FILE_FORMAT:
    case MF_E_UNSUPPORTED_SCHEME:
    case MF_E_NO_MORE_TYPES:
    case MF_E_INVALIDSTREAMNUMBER:
    case MF_E_NOT_FOUND:
    case MF_E_NOT_AVAILABLE:

        // Sample base errors
    case MF_E_NO_SAMPLE_TIMESTAMP:
    default:
    {
        //_com_error error(hr); // get the error message from the HRESULT
        //getChannel().sendMessageEvent(new ErrorInfo("mediafoundationplugin:unexpectedError", error.ErrorMessage()));
        std::cerr << "Default error" << std::endl;
        exit(1);
        return false;

    }
    }
    return true;
}

const unsigned char* MFWriter::readbinfile(std::string frame) {

    const size_t width = 320;
    const size_t height = 240;
    const size_t channels = 3;

    // calculate the size of the frame data
    const size_t framesize = width * height * channels;

    // create a vector to hold the frame data
    uint8_t* framedata = new uint8_t[framesize];

    // open the binary file
    std::ifstream file(frame, std::ios::binary);

    if (!file) {
        std::cerr << "unable to open file." << std::endl;
        exit(1);
    }

    // read the data into the vector
    file.read(reinterpret_cast<char*>(framedata), framesize * sizeof(uint8_t));

    // check if the read was successful
    if (!file) {
        std::cerr << "error reading file." << std::endl;
        exit(1);
    }

    // close the file
    file.close();

    const unsigned char* _data = reinterpret_cast<const unsigned char*>(framedata);

    return _data;
}

bool MFWriter::write(const unsigned char* const input)
{
    const size_t width = 320;
    const size_t height = 240;
    const size_t channels = 3;

    if (_sinkWriter == NULL)
    {
        HRESULT hr = createSinkWriter(static_cast<UINT32>(width), static_cast<UINT32>(height));
        // If any failure occurs after the SyncWriter is created, we need to clean it up
        // as the device is not closed. 
        if (FAILED(hr))
        {
            destroySinkWriter();
            return throwIf(hr);
        }
    }

    CComPtr<IMFMediaBuffer> mediaBuffer(createMediaBuffer(input));
    MFWRITER_RETURN_ON_ERROR(throwIf(mediaBuffer == NULL ? E_UNEXPECTED : S_OK));

    // Create a media sample and add the buffer to the sample.
    CComPtr<IMFSample> sample;
    MFWRITER_RETURN_ON_ERROR(throwIf(MFCreateSample(&sample)));
    MFWRITER_RETURN_ON_ERROR(throwIf(sample->AddBuffer(mediaBuffer)));

    // Set the time stamp and the duration.
    MFWRITER_RETURN_ON_ERROR(throwIf(sample->SetSampleTime(_frameStart)));
    MFWRITER_RETURN_ON_ERROR(throwIf(sample->SetSampleDuration(_frameDuration)));

    LONGLONG _ts;
    sample->GetSampleTime(&_ts);

    // Send the sample to the Sink Writer.
    MFWRITER_RETURN_ON_ERROR(throwIf(_sinkWriter->WriteSample(_videoStreamIndex, sample)));

    LONGLONG _ts1;
    sample->GetSampleTime(&_ts1);

    // Advance the starting position for the next frame
    _frameStart += _frameDuration;

    return true;
}

HRESULT MFWriter::createSinkWriter(UINT32 width, UINT32 height)
{
    CComPtr<IMFMediaType> pMediaTypeOut;
    CComPtr<IMFMediaType> pMediaTypeIn;

    //std::wstring wideFile = fl::filesystem::codecvt_ustring_to_filesystem().convert(_fileName);
    std::wstring wideFile = L"sample.mp4";
    

    MFWRITER_RETURN_HRESULT_ON_ERROR(MFCreateSinkWriterFromURL(wideFile.c_str(), NULL, NULL, &_sinkWriter));

    // Set the output media type.
    MFWRITER_RETURN_HRESULT_ON_ERROR(MFCreateMediaType(&pMediaTypeOut));
    MFWRITER_RETURN_HRESULT_ON_ERROR(pMediaTypeOut->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
    MFWRITER_RETURN_HRESULT_ON_ERROR(pMediaTypeOut->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_H264));

    MFWRITER_RETURN_HRESULT_ON_ERROR(pMediaTypeOut->SetUINT32(MF_MT_MPEG2_PROFILE, eAVEncH264VProfile_Main));

    // Set the Bit rate.  NOTE this assumes the input data is natively 24 bits per pixel (8 per band, 3 bands)
    MFWRITER_RETURN_HRESULT_ON_ERROR(pMediaTypeOut->SetUINT32(MF_MT_AVG_BITRATE, computeH264BitRate(_quality, width, height, _frameRate, 24)));

    MFWRITER_RETURN_HRESULT_ON_ERROR(pMediaTypeOut->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive));

    MFWRITER_RETURN_HRESULT_ON_ERROR(MFSetAttributeSize(pMediaTypeOut, MF_MT_FRAME_SIZE, width, height));

    // @todo: Frame rate should MFAverageTimePerFrameToFrameRate 
    // to allow for common frame rates like 29.97, 59.94, etc
    // Calculate the average time per frame in 100 nanosecond units
    //_frameDuration = boost::math::llround((1.0 / _frameRate) * HUNDRED_NANOSECONDS);
    _frameDuration = static_cast<long long>(std::round((1.0 / _frameRate) * HUNDRED_NANOSECONDS));

    // Derive the frame rate from 
    UINT32 rateNumerator = 0, rateDenominator = 0;
    MFWRITER_RETURN_HRESULT_ON_ERROR(MFAverageTimePerFrameToFrameRate(_frameDuration, &rateNumerator, &rateDenominator));

    MFWRITER_RETURN_HRESULT_ON_ERROR(MFSetAttributeRatio(pMediaTypeOut, MF_MT_FRAME_RATE, rateNumerator, rateDenominator));

    MFWRITER_RETURN_HRESULT_ON_ERROR(MFSetAttributeRatio(pMediaTypeOut, MF_MT_PIXEL_ASPECT_RATIO, 1, 1));
    MFWRITER_RETURN_HRESULT_ON_ERROR(_sinkWriter->AddStream(pMediaTypeOut, &_videoStreamIndex));


    // Set the input media type.
    MFWRITER_RETURN_HRESULT_ON_ERROR(MFCreateMediaType(&pMediaTypeIn));
    MFWRITER_RETURN_HRESULT_ON_ERROR(pMediaTypeIn->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
    MFWRITER_RETURN_HRESULT_ON_ERROR(pMediaTypeIn->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_YUY2));

    MFWRITER_RETURN_HRESULT_ON_ERROR(pMediaTypeIn->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive));
    MFWRITER_RETURN_HRESULT_ON_ERROR(MFSetAttributeSize(pMediaTypeIn, MF_MT_FRAME_SIZE, width, height));
    MFWRITER_RETURN_HRESULT_ON_ERROR(MFSetAttributeRatio(pMediaTypeIn, MF_MT_FRAME_RATE, rateNumerator, rateDenominator));
    MFWRITER_RETURN_HRESULT_ON_ERROR(MFSetAttributeRatio(pMediaTypeIn, MF_MT_PIXEL_ASPECT_RATIO, 1, 1));

    MFWRITER_RETURN_HRESULT_ON_ERROR(_sinkWriter->SetInputMediaType(_videoStreamIndex, pMediaTypeIn, NULL));


    _frameStart = 0;

    // Tell the sink writer to start accepting data.
    MFWRITER_RETURN_HRESULT_ON_ERROR(_sinkWriter->BeginWriting());

    return true;
}

void MFWriter::destroySinkWriter()
{
    if (_sinkWriter == NULL)
    {
        return;
    }

    _sinkWriter->Finalize();
    _sinkWriter.Release();
    _sinkWriter = NULL;
    _frameStart = 0;
    _frameDuration = 0;
}




namespace
{
    CComPtr<IMFMediaBuffer> createMediaBuffer(const unsigned char* const input)
    {
        const size_t width = 320;
        const size_t height = 240;
        const size_t channels = 3;

        // Create a new memory buffer.
        CComPtr<IMFMediaBuffer> outputBuffer;
        size_t totalBytesInBuffer = height * width * channels * 1;
        HRESULT hr = MFCreateMemoryBuffer(static_cast<DWORD>(totalBytesInBuffer), &outputBuffer);
        if (FAILED(hr))
        {
            return NULL;
        }

        // Lock the buffer and copy the video frame to the buffer.
        // Obtain the pointer to the raw buffer
        BYTE* outputData = NULL;
        hr = outputBuffer->Lock(&outputData, NULL, NULL);
        if (FAILED(hr))
        {
            return NULL;
        }

        // Copy input data to the Media Sample
        // As the input buffer contains data in YUY2 format, the effective width of the frame is doubled.
        const LONG bytesPerRow = static_cast<LONG>(2.0 * width);

        hr = MFCopyImage(outputData,                                             // Destination buffer.
            bytesPerRow,                                            // Destination stride (bytes per row).
            reinterpret_cast<const BYTE*>(input), // First row in source image.
            bytesPerRow,                                            // Source stride.
            bytesPerRow,                                            // Image width in bytes.
            static_cast<DWORD>(height));        // Image height in pixels.

        if (FAILED(hr))
        {
            return NULL;
        }

        outputBuffer->Unlock();

        // Set the data length of the buffer after writing.
        hr = outputBuffer->SetCurrentLength(static_cast<DWORD>(totalBytesInBuffer));
        if (FAILED(hr))
        {
            return NULL;
        }

        return outputBuffer;
    }

    uint32_t computeH264BitRate(const uint32_t quality, uint32_t width, uint32_t height,
        double frameRate, size_t bitsPerPixel)
    {
        // Compute a normalized bitrate (Between 0 and 1) based upon a
        // cubic function.  The coefficients were computed by analyzing
        // the output of a popular H.264 encoder at different quality
        // settings, and then using Curve Fitting Toolbox's cftool to
        // derive this equation.
        double p1 = 1.377e-06;
        double p2 = -8.498e-05;
        double p3 = 0.003415;
        double p4 = 0.01996;
        double x = quality;

        double normalizedRate = p1 * pow(x, 3) + p2 * pow(x, 2) + p3 * x + p4;

        // Compute the maximum data rate of the uncompressed video stream
        // in bits per second.
        double nativeBitsPerSecond = width * height * bitsPerPixel * frameRate;

        // Clamp the Maximum bitrate using this compression factor.
        // This value was derived by computing the ratio of uncompressed
        // data to compressed output of a popular H.264 encoder across
        // all quality values.  
        const double compressionFactor = 0.05;
        double bitrate = normalizedRate * (nativeBitsPerSecond * compressionFactor);

        if (bitrate < 0 || bitrate > static_cast<double>(std::numeric_limits<uint32_t>::max())) {
            // Conversion failed, most likely because the bitrate was too high
            return std::numeric_limits<uint32_t>::max();
        }

        return static_cast<uint32_t>(bitrate);
    }
}


int main() {
    MFWriter* _mfwrite = new MFWriter();
    std::cout << "Created MFWriter object" << std::endl;

    _mfwrite->init();
    std::cout << "MFWriter Initialized" << std::endl;

    _mfwrite->open();
    std::cout << "mfwriter opened" << std::endl;

    _mfwrite->write(_mfwrite->readbinfile("frame1.bin"));
    std::cout << "Frame 1 written" << std::endl;

    _mfwrite->write(_mfwrite->readbinfile("frame2.bin"));
    std::cout << "Frame 2 written" << std::endl;

    std::cout << "Closing object" << std::endl;
    _mfwrite->close();
    std::cout << "MFWriter Object closed" << std::endl;


    return 0;
}
