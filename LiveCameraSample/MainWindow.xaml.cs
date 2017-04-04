// 
// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license.
// 
// Microsoft Cognitive Services: http://www.microsoft.com/cognitive
// 
// Microsoft Cognitive Services Github:
// https://github.com/Microsoft/Cognitive
// 
// Copyright (c) Microsoft Corporation
// All rights reserved.
// 
// MIT License:
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED ""AS IS"", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// 

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Diagnostics;
using System.Windows.Threading;
using Newtonsoft.Json;
using OpenCvSharp;
using OpenCvSharp.Extensions;
using Microsoft.ProjectOxford.Emotion;
using Microsoft.ProjectOxford.Emotion.Contract;
using Microsoft.ProjectOxford.Face;
using Microsoft.ProjectOxford.Face.Contract;
using Microsoft.ProjectOxford.Vision;
using VideoFrameAnalyzer;

using System.IO.Ports;

namespace LiveCameraSample
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    /// 
    public partial class MainWindow : System.Windows.Window
    {
        private EmotionServiceClient _emotionClient = null;
        private FaceServiceClient _faceClient = null;
        private VisionServiceClient _visionClient = null;
        private readonly FrameGrabber<LiveCameraResult> _grabber = null;
        private static readonly ImageEncodingParam[] s_jpegParams = {
            new ImageEncodingParam(ImwriteFlags.JpegQuality, 60)
        };
        private readonly CascadeClassifier _localFaceDetector = new CascadeClassifier();
        private bool _fuseClientRemoteResults;
        private LiveCameraResult _latestResultsToDisplay = null;
        private AppMode _mode;
        private DateTime _startTime;

        public enum AppMode
        {
            Emotions,
            EmotionsWithClientFaceDetect,
            Faces,
            Tags,
            Celebrities
        }

        static List<string> happyUrl = new List<string> { "http://i.imgur.com/bhaQP9b.jpg", "http://i.imgur.com/kTuFYYO.jpg", "http://assets.nydailynews.com/polopoly_fs/1.2660532.1464999851!/img/httpImage/image.jpg_gen/derivatives/article_750/dem-2016-clinton.jpg", "http://cdn.skim.gs/images/c_fill,dpr_1.0,h_391,w_695/v1/msi/heyhuwoalyydeuaa6vsa/happy-birthday-obama" };
        static List<string> sadUrl = new List<string> { "http://i.imgur.com/kOKIzr0.jpg", "http://i.imgur.com/vWKL8OF.jpg", "http://www.cashill.com/images/Hillary_cries2.jpg", "http://www.texasfred.net/wp-content/uploads/2012/08/Sad-Obama2.jpg" };
        static List<string> surpriseUrl = new List<string> { "http://i.imgur.com/lFOnY4z.jpg", "http://i.imgur.com/WGpnuYW.jpg", "http://static.conservativetribune.com/wp-content/uploads/2016/01/Hillary-Clinton-Shocked-3.jpg", "http://static5.businessinsider.com/image/5420b1076bb3f7ff296cdbab-480/barack-obama-surprised.jpg" };
        static List<string> angryUrl = new List<string> { "http://i.imgur.com/4JcmtZJ.jpg", "http://i.imgur.com/PYfrHU9.jpg", "http://cdn.dailyheadlines.net/wp-content/uploads/2016/03/angry-hillary1.jpg", "https://conservativedailypost-guvbvzsunddro8yrw.netdna-ssl.com/wp-content/uploads/2016/06/obama-angry.jpg" };
        static List<string> disgustUrl = new List<string> { "http://i.imgur.com/Ahn8uwq.jpg", "http://imgur.com/A4mphSD.jpg","http://imgur.com/CDVKvrg.jpg", "http://imgur.com/uCkigK6.jpg" };
        static List<string> contemptUrl = new List<string> { "http://imgur.com/f5iLM3A.jpg", "https://i.ytimg.com/vi/WlrCfzI55io/maxresdefault.jpg", "https://s-media-cache-ak0.pinimg.com/736x/70/bc/60/70bc6096dcfab125e088414db5033c81.jpg", "http://i2.kym-cdn.com/entries/icons/original/000/005/498/1300044776986.jpg" };
        static List<string> fearUrl = new List<string> { "https://twistvox.files.wordpress.com/2014/05/scared-woman-014.jpg", "http://i.imgur.com/u78hYTS.jpg", "http://imgur.com/bzLgM4n.jpg", "http://imgur.com/O1fRsEP.jpg" };

        static List<string> targetList = new List<string> { "Happiness", "Sadness", "Anger", "Surprise","Disgust","Fear","Contempt" };
        static Dictionary<string,List<string>> targets = new Dictionary<string, List<string>>();
        static Random rnd = new Random(Guid.NewGuid().GetHashCode());

        static string target = "Happiness";
        static SerialPort port;

        System.Windows.Threading.DispatcherTimer dispatcherTimer = new System.Windows.Threading.DispatcherTimer();
        static int count = 5;
        static bool intermediate = false;
        static string curPic = "http://i.imgur.com/bhaQP9b.jpg";
        static bool wait = true;
        public MainWindow()
        {
            InitializeComponent();
            
            // Create grabber. 
            _grabber = new FrameGrabber<LiveCameraResult>();

            // Set up a listener for when the client receives a new frame.
            _grabber.NewFrameProvided += (s, e) =>
            {
                if (_mode == AppMode.EmotionsWithClientFaceDetect)
                {
                    // Local face detection. 
                    var rects = _localFaceDetector.DetectMultiScale(e.Frame.Image);
                    // Attach faces to frame. 
                    e.Frame.UserData = rects;
                }

                // The callback may occur on a different thread, so we must use the
                // MainWindow.Dispatcher when manipulating the UI. 
                this.Dispatcher.BeginInvoke((Action)(() =>
                {
                    // Display the image in the left pane.
                    LeftImage.Source = e.Frame.Image.ToBitmapSource();
                }));

                // See if auto-stop should be triggered. 
                if (Properties.Settings.Default.AutoStopEnabled && (DateTime.Now - _startTime) > Properties.Settings.Default.AutoStopTime)
                {
                    _grabber.StopProcessingAsync();
                }
            };

            // Set up a listener for when the client receives a new result from an API call. 
            _grabber.NewResultAvailable += (s, e) =>
            {
                this.Dispatcher.BeginInvoke((Action)(() =>
                {
                    if (e.TimedOut)
                    {
                        MessageArea.Text = "API call timed out.";
                    }
                    else if (e.Exception != null)
                    {
                        string apiName = "";
                        string message = e.Exception.Message;
                        var faceEx = e.Exception as FaceAPIException;
                        var emotionEx = e.Exception as Microsoft.ProjectOxford.Common.ClientException;
                        var visionEx = e.Exception as Microsoft.ProjectOxford.Vision.ClientException;
                        if (faceEx != null)
                        {
                            apiName = "Face";
                            message = faceEx.ErrorMessage;
                        }
                        else if (emotionEx != null)
                        {
                            apiName = "Emotion";
                            message = emotionEx.Error.Message;
                        }
                        else if (visionEx != null)
                        {
                            apiName = "Computer Vision";
                            message = visionEx.Error.Message;
                        }
                        MessageArea.Text = string.Format("{0} API call failed on frame {1}. Exception: {2}", apiName, e.Frame.Metadata.Index, message);
                    }
                    else
                    {
                        _latestResultsToDisplay = e.Analysis;

                        PlayGame(e.Frame);

                    }
                }));
            };

            // Create local face detector. 
            _localFaceDetector.Load("Data/haarcascade_frontalface_alt2.xml");
        }

        /// <summary> Function which submits a frame to the Face API. </summary>
        /// <param name="frame"> The video frame to submit. </param>
        /// <returns> A <see cref="Task{LiveCameraResult}"/> representing the asynchronous API call,
        ///     and containing the faces returned by the API. </returns>
        private async Task<LiveCameraResult> FacesAnalysisFunction(VideoFrame frame)
        {
            // Encode image. 
            var jpg = frame.Image.ToMemoryStream(".jpg", s_jpegParams);
            // Submit image to API. 
            var attrs = new List<FaceAttributeType> { FaceAttributeType.Age,
                FaceAttributeType.Gender, FaceAttributeType.HeadPose };
            var faces = await _faceClient.DetectAsync(jpg, returnFaceAttributes: attrs);
            // Count the API call. 
            Properties.Settings.Default.FaceAPICallCount++;
            // Output. 
            return new LiveCameraResult { Faces = faces };
        }

        /// <summary> Function which submits a frame to the Emotion API. </summary>
        /// <param name="frame"> The video frame to submit. </param>
        /// <returns> A <see cref="Task{LiveCameraResult}"/> representing the asynchronous API call,
        ///     and containing the emotions returned by the API. </returns>
        private async Task<LiveCameraResult> EmotionAnalysisFunction(VideoFrame frame)
        {
            // Encode image. 
            var jpg = frame.Image.ToMemoryStream(".jpg", s_jpegParams);
            // Submit image to API. 
            Emotion[] emotions = null;

            // See if we have local face detections for this image.
            var localFaces = (OpenCvSharp.Rect[])frame.UserData;
            if (localFaces != null)
            {
                // If we have local face detections, we can call the API with them. 
                // First, convert the OpenCvSharp rectangles. 
                var rects = localFaces.Select(
                    f => new Microsoft.ProjectOxford.Common.Rectangle
                    {
                        Left = f.Left,
                        Top = f.Top,
                        Width = f.Width,
                        Height = f.Height
                    });
                emotions = await _emotionClient.RecognizeAsync(jpg, rects.ToArray());
            }
            else
            {
                // If not, the API will do the face detection. 
                emotions = await _emotionClient.RecognizeAsync(jpg);
            }

            // Count the API call. 
            Properties.Settings.Default.EmotionAPICallCount++;
            // Output. 

            return new LiveCameraResult
            {
                Faces = emotions.Select(e => CreateFace(e.FaceRectangle)).ToArray(),
                // Extract emotion scores from results. 
                EmotionScores = emotions.Select(e => e.Scores).ToArray()
            };
        }

        /// <summary> Function which submits a frame to the Computer Vision API for tagging. </summary>
        /// <param name="frame"> The video frame to submit. </param>
        /// <returns> A <see cref="Task{LiveCameraResult}"/> representing the asynchronous API call,
        ///     and containing the tags returned by the API. </returns>
        private async Task<LiveCameraResult> TaggingAnalysisFunction(VideoFrame frame)
        {
            // Encode image. 
            var jpg = frame.Image.ToMemoryStream(".jpg", s_jpegParams);
            // Submit image to API. 
            var analysis = await _visionClient.GetTagsAsync(jpg);
            // Count the API call. 
            Properties.Settings.Default.VisionAPICallCount++;
            // Output. 
            return new LiveCameraResult { Tags = analysis.Tags };
        }

        /// <summary> Function which submits a frame to the Computer Vision API for celebrity
        ///     detection. </summary>
        /// <param name="frame"> The video frame to submit. </param>
        /// <returns> A <see cref="Task{LiveCameraResult}"/> representing the asynchronous API call,
        ///     and containing the celebrities returned by the API. </returns>
        private async Task<LiveCameraResult> CelebrityAnalysisFunction(VideoFrame frame)
        {
            // Encode image. 
            var jpg = frame.Image.ToMemoryStream(".jpg", s_jpegParams);
            // Submit image to API. 
            var result = await _visionClient.AnalyzeImageInDomainAsync(jpg, "celebrities");
            // Count the API call. 
            Properties.Settings.Default.VisionAPICallCount++;
            // Output. 
            var celebs = JsonConvert.DeserializeObject<CelebritiesResult>(result.Result.ToString()).Celebrities;
            return new LiveCameraResult
            {
                // Extract face rectangles from results. 
                Faces = celebs.Select(c => CreateFace(c.FaceRectangle)).ToArray(),
                // Extract celebrity names from results. 
                CelebrityNames = celebs.Select(c => c.Name).ToArray()
            };
        }

        private void PlayGame(VideoFrame frame)
        {
            var result = _latestResultsToDisplay;

            if (result != null)
            {
                if (result.EmotionScores.Length > 0 && !intermediate && !wait)
                {
                    float emotionScore = Aggregation.GetEmotion(result.EmotionScores[0], target);
                    double power = Math.Floor(emotionScore*5);
                    //port.Write(Convert.ToInt32(power).ToString());
                    switch (Convert.ToInt32(power))
                    {
                        case 0:
                            display_Box.Text = "Your power: 0000";
                            break;
                        case 1:
                            display_Box.Text = "Your power: *000";
                            break;
                        case 2:
                            display_Box.Text = "Your power: **00";
                            break;
                        case 3:
                            display_Box.Text = "Your power: ***0";
                            break;
                        case 4:
                            display_Box.Text = "Your power: ****";
                            break;
                    }
                    count = 2;
                    intermediate = true;
                }
            }
        }

        private BitmapSource VisualizeResult(VideoFrame frame)
        {
            // Draw any results on top of the image. 
            BitmapSource visImage = frame.Image.ToBitmapSource();

            var result = _latestResultsToDisplay;

            if (result != null)
            {
                // See if we have local face detections for this image.
                var clientFaces = (OpenCvSharp.Rect[])frame.UserData;
                if (clientFaces != null && result.Faces != null)
                {
                    // If so, then the analysis results might be from an older frame. We need to match
                    // the client-side face detections (computed on this frame) with the analysis
                    // results (computed on the older frame) that we want to display. 
                    MatchAndReplaceFaceRectangles(result.Faces, clientFaces);
                }

                visImage = Visualization.DrawFaces(visImage, result.Faces, result.EmotionScores, result.CelebrityNames);
                visImage = Visualization.DrawTags(visImage, result.Tags);


            }

            return visImage;
        }

        /// <summary> Populate CameraList in the UI, once it is loaded. </summary>
        /// <param name="sender"> Source of the event. </param>
        /// <param name="e">      Routed event information. </param>
        private void CameraList_Loaded(object sender, RoutedEventArgs e)
        {
            int numCameras = _grabber.GetNumCameras();

            if (numCameras == 0)
            {
                MessageArea.Text = "No cameras found!";
            }

            var comboBox = sender as ComboBox;
            comboBox.ItemsSource = Enumerable.Range(0, numCameras).Select(i => string.Format("Camera {0}", i + 1));
            comboBox.SelectedIndex = 0;
        }

        /// <summary> Populate ModeList in the UI, once it is loaded. </summary>
        /// <param name="sender"> Source of the event. </param>
        /// <param name="e">      Routed event information. </param>
        private void ModeList_Loaded(object sender, RoutedEventArgs e)
        {
            var modes = (AppMode[])Enum.GetValues(typeof(AppMode));

            var comboBox = sender as ComboBox;
            comboBox.ItemsSource = modes.Select(m => m.ToString());
            comboBox.SelectedIndex = 0;
        }

        private void ModeList_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            // Disable "most-recent" results display. 
            _fuseClientRemoteResults = false;

            var comboBox = sender as ComboBox;
            var modes = (AppMode[])Enum.GetValues(typeof(AppMode));
            _mode = modes[comboBox.SelectedIndex];
            switch (_mode)
            {
                case AppMode.Faces:
                    _grabber.AnalysisFunction = FacesAnalysisFunction;
                    break;
                case AppMode.Emotions:
                    _grabber.AnalysisFunction = EmotionAnalysisFunction;
                    break;
                case AppMode.EmotionsWithClientFaceDetect:
                    // Same as Emotions, except we will display the most recent faces combined with
                    // the most recent API results. 
                    _grabber.AnalysisFunction = EmotionAnalysisFunction;
                    _fuseClientRemoteResults = true;
                    break;
                case AppMode.Tags:
                    _grabber.AnalysisFunction = TaggingAnalysisFunction;
                    break;
                case AppMode.Celebrities:
                    _grabber.AnalysisFunction = CelebrityAnalysisFunction;
                    break;
                default:
                    _grabber.AnalysisFunction = null;
                    break;
            }
        }

        private async void StartButton_Click(object sender, RoutedEventArgs e)
        {
            if (!CameraList.HasItems)
            {
                MessageArea.Text = "No cameras found; cannot start processing";
                return;
            }

            if ((_mode == AppMode.EmotionsWithClientFaceDetect || _mode == AppMode.Emotions) /*&& textBox.Text == "open"*/)
            {
                SettingsPanel.Visibility = Visibility.Hidden;
                targets["Happiness"] = happyUrl;
                targets["Sadness"] = sadUrl;
                targets["Surprise"] = surpriseUrl;
                targets["Anger"] = angryUrl;
                targets["Disgust"] = disgustUrl;
                targets["Contempt"] = contemptUrl;
                targets["Fear"] = fearUrl;
                curPic = targets[target][rnd.Next(0,4)];
                RightImage.Source = new BitmapImage(new Uri(curPic));
                dispatcherTimer.Tick += new EventHandler(dispatcherTimer_Tick);
                dispatcherTimer.Interval = new TimeSpan(0, 0, 1);
                dispatcherTimer.Start();
                display_Box.Text = target;
                // Clean leading/trailing spaces in API keys. 
                Properties.Settings.Default.FaceAPIKey = Properties.Settings.Default.FaceAPIKey.Trim();
                Properties.Settings.Default.EmotionAPIKey = Properties.Settings.Default.EmotionAPIKey.Trim();
                Properties.Settings.Default.VisionAPIKey = Properties.Settings.Default.VisionAPIKey.Trim();

                // Create API clients. 
                _faceClient = new FaceServiceClient(Properties.Settings.Default.FaceAPIKey);
                _emotionClient = new EmotionServiceClient(Properties.Settings.Default.EmotionAPIKey);
                _visionClient = new VisionServiceClient(Properties.Settings.Default.VisionAPIKey);

                // How often to analyze. 
                _grabber.TriggerAnalysisOnInterval(Properties.Settings.Default.AnalysisInterval);

                // Reset message. 
                MessageArea.Text = "";

                // Record start time, for auto-stop
                _startTime = DateTime.Now;

                await _grabber.StartProcessingCameraAsync(CameraList.SelectedIndex);
            }
        }

        private async void StopButton_Click(object sender, RoutedEventArgs e)
        {
            await _grabber.StopProcessingAsync();
            dispatcherTimer.Stop();
            count = 0;
            display_Box.Text = "Off";
            textBox2.Text = count.ToString();
        }

        private void SettingsButton_Click(object sender, RoutedEventArgs e)
        {
            SettingsPanel.Visibility = 1 - SettingsPanel.Visibility;
        }

        private void SaveSettingsButton_Click(object sender, RoutedEventArgs e)
        {   
            SettingsPanel.Visibility = Visibility.Hidden;
            Properties.Settings.Default.Save();
        }

        private void Hyperlink_RequestNavigate(object sender, RequestNavigateEventArgs e)
        {
            Process.Start(new ProcessStartInfo(e.Uri.AbsoluteUri));
            e.Handled = true;
        }

        private Face CreateFace(FaceRectangle rect)
        {
            return new Face
            {
                FaceRectangle = new FaceRectangle
                {
                    Left = rect.Left,
                    Top = rect.Top,
                    Width = rect.Width,
                    Height = rect.Height
                }
            };
        }

        private Face CreateFace(Microsoft.ProjectOxford.Vision.Contract.FaceRectangle rect)
        {
            return new Face
            {
                FaceRectangle = new FaceRectangle
                {
                    Left = rect.Left,
                    Top = rect.Top,
                    Width = rect.Width,
                    Height = rect.Height
                }
            };
        }

        private Face CreateFace(Microsoft.ProjectOxford.Common.Rectangle rect)
        {
            return new Face
            {
                FaceRectangle = new FaceRectangle
                {
                    Left = rect.Left,
                    Top = rect.Top,
                    Width = rect.Width,
                    Height = rect.Height
                }
            };
        }

        private void MatchAndReplaceFaceRectangles(Face[] faces, OpenCvSharp.Rect[] clientRects)
        {
            // Use a simple heuristic for matching the client-side faces to the faces in the
            // results. Just sort both lists left-to-right, and assume a 1:1 correspondence. 

            // Sort the faces left-to-right. 
            var sortedResultFaces = faces
                .OrderBy(f => f.FaceRectangle.Left + 0.5 * f.FaceRectangle.Width)
                .ToArray();

            // Sort the clientRects left-to-right.
            var sortedClientRects = clientRects
                .OrderBy(r => r.Left + 0.5 * r.Width)
                .ToArray();

            // Assume that the sorted lists now corrrespond directly. We can simply update the
            // FaceRectangles in sortedResultFaces, because they refer to the same underlying
            // objects as the input "faces" array. 
            for (int i = 0; i < Math.Min(faces.Length, clientRects.Length); i++)
            {
                // convert from OpenCvSharp rectangles
                OpenCvSharp.Rect r = sortedClientRects[i];
                sortedResultFaces[i].FaceRectangle = new FaceRectangle { Left = r.Left, Top = r.Top, Width = r.Width, Height = r.Height };
            }
        }

        

        private void NewTargetButton_Click(object sender, RoutedEventArgs e)
        {
            target = targetList[rnd.Next(0,7)];
        }


        private void name_button_Click(object sender, RoutedEventArgs e)
        {


            string name = name_box.Text;
            int baud = 9600;
            BeginSerial(baud, name);
            port.Open();

            textBox.Text = "open";

        }

        private void name_button_click_Close(object sender, RoutedEventArgs e)
        {

                    port.Close();
                textBox.Text = "closed";

        }
        private void dispatcherTimer_Tick(object sender, EventArgs e)
        {
            if(count > 0)
                count = count - 1;
            textBox2.Text = count.ToString();

            if (wait)
            {
                display_Box_bottom.Text = count.ToString();
            }
            if (count == 0)
            {
                if (intermediate)
                {
                    count = 3;
                    target = targetList[rnd.Next(0, 7)];
                    display_Box.Text = target;
                    curPic = targets[target][rnd.Next(0, 4)];
                    RightImage.Source = new BitmapImage(new Uri(curPic));
                    intermediate = false;
                    wait = true;
                }

                else if (wait)
                {
                    display_Box_bottom.Text = "";
                    wait = false;
                }
            }
        }
        
        private void timer_button_click(object sender, RoutedEventArgs e)
        {
            dispatcherTimer.Tick += new EventHandler(dispatcherTimer_Tick);
            dispatcherTimer.Interval = new TimeSpan(0, 0, 1);
            dispatcherTimer.Start();
        }
        

        static void port_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            for (int i = 0; i < (10000 * port.BytesToRead) / port.BaudRate; i++)
                ;       //Delay a bit for the serial to catch up
            //Console.Write(port.ReadExisting());
        }

        static void BeginSerial(int baud, string name)
        {
            port = new SerialPort(name, baud);
        }

        static int GetBaudRate()
        {
            try
            {
                return int.Parse(Console.ReadLine());
            }
            catch
            {
                Console.WriteLine("Invalid integer.  Please try again:");
                return GetBaudRate();
            }
        }
    }
}
