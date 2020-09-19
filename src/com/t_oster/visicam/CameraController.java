package com.t_oster.visicam;
import org.bytedeco.javacv.FrameGrabber.Exception;
import org.bytedeco.javacv.OpenCVFrameGrabber;
import static org.bytedeco.javacpp.opencv_calib3d.*;
import static org.bytedeco.javacpp.opencv_core.*;
import org.bytedeco.javacpp.opencv_core.CvMat;
import org.bytedeco.javacpp.opencv_core.CvMemStorage;
import org.bytedeco.javacpp.opencv_core.CvPoint;
import org.bytedeco.javacpp.opencv_core.CvPoint2D32f;
import org.bytedeco.javacpp.opencv_core.CvPoint3D32f;
import org.bytedeco.javacpp.opencv_core.CvSeq;
import org.bytedeco.javacpp.opencv_core.IplImage;
import static org.bytedeco.javacpp.opencv_imgproc.*;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Rectangle;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.PrintWriter;
import java.io.RandomAccessFile;
import java.nio.channels.FileLock;
import java.nio.channels.FileChannel;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.Path;
import javax.imageio.ImageIO;
import org.bytedeco.javacpp.BytePointer;

/**
 *
 * @author Thomas Oster <thomas.oster@rwth-aachen.de>
 */
public class CameraController 
{
  // Variables to support grid-based distortion correction
  private final Integer gridMatrixRows = 4;
  private final Integer gridMatrixCols = 4;
  private final Integer gridMatrixCount = gridMatrixRows * gridMatrixCols;
  private volatile CvMat[] gridHomographyMatrixes = null;

  private volatile CvMat homographyMatrix = null;
  private final Boolean synchronizedCamera = false;                          // Dummy variable only used for camera access synchronization
  private final Boolean synchronizedMatrix = false;                          // Dummy variable only used for matrix access synchronization

  // visicamRPiGPU integration start
  private final Boolean visicamRPiGPUFileLockSynchronization = false;        // Dummy variable only used for file lock synchronization
  // visicamRPiGPU integration end

  public InputStream toJpegStream(final BufferedImage img) throws IOException
  {
    ByteArrayOutputStream bos = new ByteArrayOutputStream();
    ImageIO.write(img, "jpg", bos);
    return new ByteArrayInputStream(bos.toByteArray());
  }

  public synchronized BufferedImage takeSnapshot(int cameraIndex, int width, int height, String command, String path, boolean visicamRPiGPUEnabled, String visicamRPiGPUImageOriginalPath) throws Exception, IOException, InterruptedException
  {
    BufferedImage result;

    // visicamRPiGPU integration start
    if (visicamRPiGPUEnabled)
    {
        // Default return value
        result = null;

        // Create file object
        File originalImageFile = new File(visicamRPiGPUImageOriginalPath);

        // Check if file exists
        if (originalImageFile.exists() && !originalImageFile.isDirectory())
        {
            // Variable to store result
            byte[] originalImageFileData = null;

            // Synchronized access because file lock may only be acquired once by this Java application
            synchronized (visicamRPiGPUFileLockSynchronization)
            {
                // Lock file
                FileChannel originalImageChannel = new RandomAccessFile(originalImageFile, "rw").getChannel();
                FileLock originalImageLock = originalImageChannel.lock();

                // Read file data into memory
                Path originalImagePath = Paths.get(visicamRPiGPUImageOriginalPath);
                originalImageFileData = Files.readAllBytes(originalImagePath);

                // Unlock, close file
                originalImageLock.release();
                originalImageChannel.close();
            }

            // Create input stream from memory file data
            ByteArrayInputStream originalImageByteInputStream = new ByteArrayInputStream(originalImageFileData);

            // Build result from input stream
            result = ImageIO.read(originalImageByteInputStream);
        }
    }
    // visicamRPiGPU integration end
    else if (command == null || "".equals(command))
      {
        synchronized (synchronizedCamera)
        {
            OpenCVFrameGrabber grabber = new OpenCVFrameGrabber(cameraIndex);
            grabber.setImageHeight(height);
            grabber.setImageWidth(width);
            grabber.start();
            IplImage img = grabber.grab();
            result = img.getBufferedImage();
            grabber.stop();
            if (result == null) {
              throw new Exception("Grabbing image failed (result is null). Something is very wrong with your camera.");
            }
        }
      }
      else
      {
        synchronized (synchronizedCamera)
        {
            Runtime r = Runtime.getRuntime();
            command = command.replace("%i", ""+cameraIndex).replace("%w", ""+width).replace("%h", ""+height).replace("%f", path);
            Process pr = r.exec(command);
            InputStream os = pr.getErrorStream();
            int b;
            String errors = "";
            while  ((b = os.read()) != -1)
            {
              errors += ""+(char)b;
            }
            pr.waitFor();
            if (pr.exitValue() != 0)
            {
              String errorMessage = "Command failed (exit " + pr.exitValue();
              if ("".equals(errors)) {
                  errorMessage += ", no output on stderr)";
              } else {
                  errorMessage += "): " + errors;
              }
              throw new Exception(errorMessage);
            }
            result = ImageIO.read(new File(path));
            if (result == null) {
              throw new Exception("Fetching image failed: empty output file '" + path + "'");
            }
        }
    }

  return result;
  }

  public BufferedImage getDummyImage(String filename, String txt) throws IOException {
	BufferedImage result = ImageIO.read(new File(filename));
        Graphics2D g = result.createGraphics();
        g.setFont(g.getFont().deriveFont(72));
        double w = g.getFontMetrics().stringWidth(txt);
        int textHeight=2*g.getFontMetrics().getHeight();
        //g.scale((result.getWidth()-200)/w, (result.getWidth()-200)/w);
        g.clearRect(100, 100, result.getWidth()-200, textHeight);
        g.drawString(txt, 100, 100+textHeight);
        float s = (float) ((result.getWidth()-200)/w);
        g.scale(s,s);
        g.clearRect((int) (100/s), (int) (100/s), result.getWidth()-200, textHeight);
        g.drawString(txt, 100/s, 100/s+textHeight/2);
	return result;
  }
  
  public RelativePoint findMarker(BufferedImage input, RelativeRectangle roi) throws Exception
  {
    Rectangle abs = roi.toAbsoluteRectangle(input.getWidth(), input.getHeight());
    if (abs.width == 0 || abs.height == 0) {
        // Allow single points as fixed values
        return new RelativePoint((abs.x)/(double)input.getWidth(), (abs.y)/(double)input.getHeight());
    }
    IplImage in = IplImage.createFrom(input.getSubimage(abs.x, abs.y, abs.width, abs.height));
    IplImage gray = IplImage.create(in.width(), in.height(), in.depth(), 1);
    cvCvtColor(in, gray, CV_BGR2GRAY);
    cvErode(gray, gray, null, 1);
    cvErode(gray, gray, null, 1);
    cvDilate(gray, gray, null, 1);
    cvDilate(gray, gray, null, 1);
    cvErode(gray, gray, null, 1);
    cvDilate(gray, gray, null, 1);
    CvMemStorage storage = cvCreateMemStorage(0);
    double minDist = 10;
    double cannyThreshold = 50;
    double minVotes = 15;
    int minRad = 5;
    int maxRad = 25;
    CvSeq circles = cvHoughCircles(gray, storage, CV_HOUGH_GRADIENT, 1, minDist, cannyThreshold, minVotes, minRad, maxRad);
    if (circles.total() >= 1)
    {
      CvPoint3D32f center = new CvPoint3D32f(cvGetSeqElem(circles, 0));
      RelativePoint result = new RelativePoint((center.x()+abs.x)/(double)input.getWidth(), (center.y()+abs.y)/(double)input.getHeight());
      cvReleaseMemStorage(storage);
      return result;
    }
    cvReleaseMemStorage(storage);
    // If marker not found, use center of search region 
    VisiCam.log("Warning: Marker not found at (" + roi.x + "," + roi.y + "," + roi.w + "," + roi.h + "); using center.");
    return new RelativePoint((abs.x+abs.width/2)/(double)input.getWidth(), (abs.y+abs.height/2)/(double)input.getHeight());
  }
  
  // find all markers in searchfields, returns an array with RelativePoint entries (or null entry if marker not found)
  public RelativePoint[] findMarkers(BufferedImage img, RelativeRectangle[] markerSearchfields) throws Exception
  {
      RelativePoint[] currentMarkerPositions = new RelativePoint[markerSearchfields.length];
      for (int i = 0; i < markerSearchfields.length; i++)
      {
        currentMarkerPositions[i] = findMarker(img, markerSearchfields[i]);
        // VisiCam.log("Marker " + i + ":"  + currentMarkerPositions[i] + " (in rectangle: " + markerSearchfields[i] +  ")");
      }
      return currentMarkerPositions;
  }
  
  public BufferedImage applyHomography(BufferedImage img)
  {
        synchronized (synchronizedMatrix)
        {
            if (gridHomographyMatrixes != null)
            {
                IplImage in = IplImage.createFrom(img);
                IplImage out = IplImage.createFrom(img);
                
                int k = 0;
                int width = img.getWidth();
                int height = img.getHeight();
                for (int j=0; j<gridMatrixRows; j++) {
                    for (int i=0; i<gridMatrixCols; i++) {
                        int first = j*(gridMatrixCols+1)+i;
                        CvRect rect = cvRect(i*width/gridMatrixCols, j*height/gridMatrixRows, width/gridMatrixCols, height/gridMatrixRows);
                        
                        // Map output of each matrix to target cell
                        cvSetImageROI(out, rect); 
                        cvWarpPerspective(in, out, gridHomographyMatrixes[k++]);
                        cvResetImageROI(out); 
                    }
                }
                return out.getBufferedImage();
            }
            else if (homographyMatrix != null)
            {
                IplImage in = IplImage.createFrom(img);
                cvWarpPerspective(in, in, homographyMatrix);
                return in.getBufferedImage();
            }
            else
            {
                return null;
            }
        }
  }

  public void updateHomographyMatrix(BufferedImage img, RelativePoint[] markerPositions, float zoomOutputPercent, double outputWidth, double outputHeight, boolean visicamRPiGPUEnabled, String visicamRPiGPUMatrixPath) throws FileNotFoundException, IOException
  {
      // Simple 4-corner mapping 
      if (markerPositions.length == 4) {
          CvMat localHomographyMatrix = calculateHomographyMatrix(img, 1,1, markerPositions, zoomOutputPercent, outputWidth, outputHeight, visicamRPiGPUEnabled, visicamRPiGPUMatrixPath);
          
          // This looks weird, but homographyMatrix must not be null for synchronized access
          // If it is null, no need to care for synchronized access at all
          if (homographyMatrix != null) {
              synchronized (synchronizedMatrix) {
                  homographyMatrix = localHomographyMatrix;
              }
          } else {
              homographyMatrix = localHomographyMatrix;
          }
      } 
      
      // Grid mapping from 5x5 array of points to 4x4 grid of matrices
      else if (markerPositions.length == (gridMatrixRows+1) * (gridMatrixCols+1)) {
          CvMat[] matrixes = new CvMat[gridMatrixCount];
          
          // Generate matrixes
          int k = 0;
          for (int j=0; j<gridMatrixRows; j++) {
              for (int i=0; i<gridMatrixCols; i++) {
                  int first = j*(gridMatrixCols+1)+i;
                  RelativePoint[] corners = new RelativePoint[] { markerPositions[first], markerPositions[first+1], markerPositions[first+gridMatrixCols+1], markerPositions[first+gridMatrixCols+2] };
                  matrixes[k++] = calculateHomographyMatrix(img, gridMatrixCols, gridMatrixRows, corners, zoomOutputPercent, outputWidth, outputHeight, visicamRPiGPUEnabled, visicamRPiGPUMatrixPath);
              }
          }
          
          // Save matrixes
          if (gridHomographyMatrixes != null) {
              synchronized (synchronizedMatrix) {
                  gridHomographyMatrixes = matrixes;
              }
          } else {
              gridHomographyMatrixes = matrixes;
          }

          // Recursively call to update 4-corner matrix
          RelativePoint[] cornerMarkers = new RelativePoint[] { markerPositions[0], markerPositions[4], markerPositions[20], markerPositions[24] };
          updateHomographyMatrix(img, cornerMarkers, zoomOutputPercent, outputWidth, outputHeight, visicamRPiGPUEnabled, visicamRPiGPUMatrixPath);
      }
      
      // Error
      else {
          VisiCam.log("updateHomographyMatrix called with unsupported marker count " + markerPositions.length);
      }
  }

  public CvMat calculateHomographyMatrix(BufferedImage img, float divideX, float divideY, RelativePoint[] markerPositions, float zoomOutputPercent, double ouputWidth, double outputHeight, boolean visicamRPiGPUEnabled, String visicamRPiGPUMatrixPath) throws FileNotFoundException, IOException
  {
    CvMat src = CvMat.create(markerPositions.length, 1, CV_32FC(2));

    for (int i = 0; i < markerPositions.length; i++)
    {
      Point p = markerPositions[i].toAbsolutePoint(img.getWidth(), img.getHeight());
      src.put(i, 0, 0, p.x);
      src.put(i, 0, 1, p.y);
    }

    CvMat dst = CvMat.create(4, 1, CV_32FC(2));
    dst.put(0, 0, 0, 0);
    dst.put(0, 0, 1, 0);
    dst.put(1, 0, 0, img.getWidth());
    dst.put(1, 0, 1, 0);
    dst.put(2, 0, 0, 0);
    dst.put(2, 0, 1, img.getHeight());
    dst.put(3, 0, 0, img.getWidth());
    dst.put(3, 0, 1, img.getHeight());
    
    CvMat zoomMat = CvMat.create(3, 3);
    // [x'; y'; 1] = zoomMat * [x; y; 1]
    // x' = (x-width/2)*zoom + width/2
    // same for y'.
    // for zoom=1 (100%), the result is the identity matrix.
    zoomMat.put(0, 0, zoomOutputPercent/100.f/divideX);
    zoomMat.put(0, 1, 0);
    zoomMat.put(0, 2, img.getWidth() / 2 * (1 - zoomOutputPercent/100.f));
    // same for y
    zoomMat.put(1, 0, 0);
    zoomMat.put(1, 1, zoomOutputPercent/100.f/divideY);
    zoomMat.put(1, 2, img.getHeight() / 2 * (1 - zoomOutputPercent/100.f));
    // keep last coordinate (always 1 for affine transformation)
    zoomMat.put(2, 0, 0);
    zoomMat.put(2, 1, 0);
    zoomMat.put(2, 2, 1);

    CvMat localHomographyMatrixWithoutZoom = CvMat.create(3, 3);
    cvFindHomography(src, dst, localHomographyMatrixWithoutZoom, CV_RANSAC, 1, null);
    CvMat localHomographyMatrix = CvMat.create(3,3);
    // System.out.println(zoomMat.toString() + "\n");
    // System.out.println(localHomographyMatrixWithoutZoom.toString() + "\n");
    // matrix multiply: localHomographyMatrix = zoomMat * localHomographyMatrixWithoutZoom * 1 + 0
    cvGEMM(zoomMat, localHomographyMatrixWithoutZoom, 1, zoomMat, 0, localHomographyMatrix);
    // System.out.println(localHomographyMatrix.toString() + "\n"); 

    // Write matrix values to file for visicamRPiGPU if needed
    if (visicamRPiGPUEnabled)
    {
        // Create file object
        File matrixOutputFile = new File(visicamRPiGPUMatrixPath);

        // Check if file exists
        // Lock file
        FileChannel matrixOutputChannel = new RandomAccessFile(matrixOutputFile, "rw").getChannel();
        FileLock matrixOutputLock = matrixOutputChannel.lock();

        // Write matrix values to file
        PrintWriter matrixOutputWriter = new PrintWriter(matrixOutputFile);

        // Iterate over matrix and write data
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                float matrixValue = (float)(localHomographyMatrix.get(i, j));
                matrixOutputWriter.println(matrixValue);
            }
        }

        // Close writer
        matrixOutputWriter.flush();
        matrixOutputWriter.close();

        // Unlock, close file
        matrixOutputLock.release();
        matrixOutputChannel.close();
    }
    return localHomographyMatrix;
  }
  
  public void setHomographyMatrixInvalid()
  {
    if (homographyMatrix != null)
    {
        synchronized (synchronizedMatrix)
        {
            homographyMatrix = null;
        }
    }

    // Clear grid matrixes
    if (gridHomographyMatrixes != null) {
        synchronized (synchronizedMatrix) {
            gridHomographyMatrixes = null;
        }
    }
  }

  public CvMat getHomographyMatrix()
  {
    // homographyMatrix must not be null for synchronized access
    if (homographyMatrix != null)
    {
        synchronized (synchronizedMatrix)
        {
            return homographyMatrix;
        }
    }

    return null;
  }
}
