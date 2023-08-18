
#ifndef TRACKING_3D_IMAGESTREAM_H
#define TRACKING_3D_IMAGESTREAM_H
#include <string>
#include <iomanip>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include "../util.cpp"
#include "../datatype/data_structure.cpp"

/**
 * @class ImageStream handle streaming image data coming from sensor. Storing the stream into @DataFrame.
 */
class ImageStream
        {
private:
    std::string mainPath;               // Stream Directory.
    std::string imagePrefix;            // Image Extension.
    DataFrame* dataFrame;               // Data holder for the image.

    int currentStreamId;                // Current stream index, begins by 0.
    bool visual;                        // To visualize the internal process. [Validation/Testing]

    const int _REPEATED_WIDTH   = 6;    // Number of digits for each input file, in KITTI.
    const char _FIXED_INDEX     = '0';  // Numerical value on the data.



public:
    /**
    * @ImageStream constructor, initialize attributes.
    * @param streamMainPath Original directory of the image stream dataset.
    * @param dataframe A pointer data holder to store the recent streamed image.
    * @param prefix Image extension, expected input should be from @struct ImageStream::DataType.
    * @param bVisual To monitor the internal procedures.
    */
    ImageStream(std::string streamMainPath,  DataFrame* dataFrame, std::string imgPrefix = ImageType::IMAGE_JPG, bool bVisual = false); // Reading from certain directory.

    ImageStream();  // Reading from sensor.
    ~ImageStream();

    /**
     * This function use the provided path to read an image stream in order and store it into dataframe pointer
     * until no images are exist.
     * @return True: No file exist. False: Image file is read correctly.
     */
    bool hasNext(); // Read a stream from the given path.

    /**
     *  An @struct to initialize image extensions while reading from a stream.
     *  Acceptable extensions are .JPG, and .PNG images.
     */
    struct ImageType{
        static const std::string IMAGE_JPG;
        static const std::string IMAGE_PNG;

        /**
         * @ThrowImageExtensionException exception for the input extension to make sure
         * the input stream is as expected.
         */
        struct ThrowImageExtensionException: std::exception{
            const char* what() const throw(){
                return "Unsupported Extension";
            }
        };

        /**
         * To verify the input file extension
         * @param prefix file extension.
         */
        static void acceptable(std::string prefix)
        {
            if(prefix != IMAGE_JPG && prefix != IMAGE_PNG)
                throw ThrowImageExtensionException();
        }
    };

};



#endif //TRACKING_3D_IMAGESTREAM_H
