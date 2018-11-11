
COMMAND-LINE OPTION SYNTAX:

Simulation Control Options:

  --timestep=<double> [0.0001]  (SimTime)
      Simulation time step (in seconds) to use for various difference equations 
      in the model

  --too-much-time=<time> [unlimited]  (SimTime)
      Exit after the maximum specified time in seconds

  --seq-type=<Std|Debug> [Std]  (std::string)
      Type of event queue used to control simulations. Usually you would want 
      to use 'Std', for standard clocking and dispatching of events that arise 
      in various modules being simulated. 'Debug' provides more verbose tracing 
      of events, which is useful to debug simulation flow issues.


General-Use Options:

  -d, --[no]debug [no]
      Use debug mode, which, in particular, will increase the verbosity of the 
      log messages and printout all model parameter values just before the 
      model starts

  --[no]use-random [yes]
      Add small amounts of random noise to various stages of model

  --[no]mem-stats [no]
      Show verbose memory allocation statistics.

  --mem-stats-units=<bytes> [0]  (unsigned long)
      Allocation unit size (in bytes) to use when displaying the verbose memory 
      statistics, or 0 to let simulation modules decide on an allocation unit 
      each time they request that stats be shown.

  --textlog=<file.log> []  (std::string)
      Save text log messages to file

  -Z, --[no]test-mode [no]
      Use test mode, which, in particular, will turn off randomness (like 
      --nouse-random), reset random numbers to a reproducible pseudo sequence, 
      and turn off most displays. This is mostly useful for execution of the 
      test suite, or any situation where you need deterministic, reproducible 
      behavior

  -h, --[no]help [yes]
      Show help message and option syntax

  --[no]version [no]
      Show the package name, version, and the svn repository version of the 
      current build

  --[no]svnversion [no]
      Show the svn repository version of the current build

  --[no]check-pristine [no]
      Check if the executable was built from pristine source (i.e., all files 
      up-to-date and not locally modified), and return an exit code of zero if 
      true, and non-zero otherwise

  --[no]use-fpe [yes]
      Use floating-point exceptions, which will abort execution if overflow, 
      underflow, invalid or division by zero are encountered

  --fpu-precision=<single|double|extended> [extended]  (FpuPrecision)
      Specifies the precision in which the floating-point unit (FPU) should 
      operate; default is 'extended'

  --fpu-rounding=<nearest|down|up|zero> [nearest]  (FpuRoundingMode)
      Specifies the rounding-mode in which the floating-point unit (FPU) should 
      operate; options are nearest (toward nearest), down (toward -inf), up 
      (toward +inf), zero (toward zero)

  --profile-out=<filename> []  (std::string)
      Where to save profiling information upon program exit (if 'none' is 
      passed for the filename, then profiling information will not be saved at 
      all)

  --logverb=<Debug|Info|Error|Fatal|Default> [Default]  (std::string)
      Verbosity of log messages displayed during execution. If 'Default' is 
      selected, will use whatever value may have been set as default in the 
      program.

  --[no]echo-args [no]
      Echo all the command-line arguments to stdout after command-line parsing 
      is done.

  --[no]mem-caching [yes]
      Whether to cache memory allocations of commonly-used sizes.

  --load-config-from=<file.pmap> []  (std::string)
      Load configuration from file

  --save-config-to=<file.pmap> []  (std::string)
      Save configuration to file


Input Reading/Formatting Options:

  --zero-number-frames=<true|false> [false]  (bool)
      Force all input and output frames to have the number 000000

  --io=<intersection of valid values for --in and --out> []  (std::string)
      Specify both input and output with a single specification. Thus 
      --io=type:foo is equivalent to --in=type:foo --out=type:foo. NOTE that 
      this will only work if 'type' is valid for both input and output; 
      currently that would include 'raster' and 'mpeg', but not 'display' 
      (valid for output only) or 'random' (valid for input only).

  -M, --input-frames=[[first[-step]]-[last]]@[delay_or_rate] [0-1-MAX@30Hz]  
    (FrameRange)
      Input frame range and inter-frame delay or rate. The frame range can 
      include optional specifications for the first frame (default value=0), 
      last frame (default=MAX), and/or frame step (default=1). A fixed 
      framerate can be specified either as an inter-frame interval, with a 
      suffix one of {s,ms,us,ns}, or as a frame rate, with a suffix of Hz. A 
      variable framerate can be specified with the name of a file (.fl) that 
      contains precise frame timing information, with lines of the form:
      	<name> <time>
      where:
      	<name> is a string (no spaces) and
      	<time> is the elapsed time from stimulus onset (in ms)
      	the <time> in the first line should be 0
      	the <name> in the last line should be END
      	the <time> in the last line should mark the stimulus end time

  --[no]input-frames-wrap [no]
      Whether to set the frame number back to the start if we reached the end. 
      This needs the --input-frames to be set 

  --crop-input=x1,y1,x2,y2 [0,0,0,0]  (Rectangle)
      Crop input frames, or 0,0,0,0 for no cropping

  --rescale-input=<width>x<height> [0x0]  (Dims)
      Rescale input frames to fixed dims, or 0x0 for no rescaling

  --[no]preserve-input-aspect [no]
      Preserve input frame aspect ratio if rescaling to fixed dims

  --in=<[type]:[spec]> []  (std::string)
      Specify a source of input frames. If this option is given multiple times, 
      then only the last value will have any effect. The general expected form 
      is [type]:[spec], where [type] specifies the type of input stream (such 
      as raster files, a movie file, etc.), and [spec] provides additional 
      information for the input stream (such as the name of the input files). 
      In certain common cases, the [type] may be omitted if it can be inferred 
      from the filename dot-extension spec, as in the case where the 'png' 
      raster stream type can be inferred from the fact that the filename ends 
      in '.png'. Also, for certain stream types, the [spec] is optional and may 
      be omitted. There are several general input stream types: (1) raw input 
      sources, which represent input images being read from some external 
      source such as files on disk or input from a camera; (2) generated input 
      sources, which represent images being generated online by the program, 
      such as test stimuli; and (3) filters, which are input sources that 
      generate images by transforming images that come from some other source. 
      Filters can be chained together arbitrarily. Finally, some input sources 
      may not be supported on every system, depending on the operating system 
      and available libraries; unavailable input sources are still listed below 
      but are specifically marked as being unavailable. Following is a list of 
      possible stream types, their corresponding filename dot-extensions, and 
      the format expected of their [spec]:
      
      RAW INPUT SOURCES
              --in=raster:path/to/file#nnn#.[<width>x<height>].[ext]
                Reads input from one or more raster files. The leading 
      'raster:' may be omitted if the file has one of the known raster 
      extensions: .pnm, .pgm, .ppm, .pbm, .pfm, .png, .jpeg, .jpg, .dpx, 
      .rgb555, .rgb565, .rgb24, .rgb32, .yuv24, .yuyv, .uyvy, .yuv444, .yuv422, 
      .yuv411, .yuv420, .yuv410, .yuv444p, .yuv422p, .yuv411p, .yuv420p, 
      .yuv410p; or, any of the above with an additional .gz or .bz2 extension, 
      in which case the image file will be transparently decompressed. If the 
      given filename includes a pair of hashes ('#'), then characters in 
      between the hashes will be interpreted as a minimum numeric field width, 
      and the frame number will be formatted as a zero-padded string to that 
      minimum width, and will be substituted for the entire #nnn# sequence. As 
      special cases, a single '#' is equivalent to '#6#', and '##' is 
      equivalent to '#0#'. Thus a filename of foo#.png or foo#6#.png will cause 
      the stream to read foo000000.png, foo000001.png, etc.; foo##.png or 
      foo#0#.png will read foo0.png, foo1.png, etc.; and foo#3#.png will read 
      foo000.png, foo001.png, etc. Note that for the raw formats that have no 
      header information (e.g., rgb24, yuv420p), you need to specify the image 
      dimensions somehow; the preferred method for doing this is to encode the 
      dimensions as '.WWWxHHH' into a secondary dot-extension just prior to the 
      pixel-format filename extension (this is the same filename format 
      produced by --out=rawvideo). If no such .WWWxHHH is found in the 
      filename, then the dimensions will be taken from the value passed to 
      --yuv-dims, but this approach is deprecated.
              --in=rasterlist:listfilename
                Reads a list of raster file names from 'listfilename', with one 
      file name per line. The corresponding images will be loaded as input 
      frames, in the order in which they are listed in the list file. The 
      raster files may be in any of the file formats accepted by --in=raster. 
      Note that all of the files in the list must be of the same image type; 
      that is, they must all have the same pixel format (such as RGB, 
      grayscale, YUV420P, etc.), and they must all have the same image 
      dimensions.
              --in=movie:path/to/movie.[ext]
                Reads input from a movie file. The leading 'movie:' may be 
      omitted if the file has one of the known movie extensions: .avi, .mpg, 
      .mpeg, .m4v, .mov, .flv, or .dv.
              --in=mpeg:path/to/movie.[ext]
                Equivalent to --in=movie; present for backward compatibility 
      from when the only supported movie type was mpeg.
              --in=mgz:path/to/file.mgz
                Reads input frames from a file in our custom 'mgz' format, 
      which is essentially a single file containing a gzip-compressed sequence 
      of images. The leading 'mgz:' may be omitted if the given filename ends 
      with a '.mgz' extension.
              --in=mraw:path/to/file.[<width>x<height>].[ext]?.gz|.bz2?
                Reads multiple raw-format input frames from a single file. 
      Because this is a truly raw file format containing no header or 
      formatting information, you must provide some auxiliary information by 
      encoding it into the filename. Specifically, the file must end with two 
      or three dot-extensions. The first dot-extension specifies the width and 
      height dimensions of the video frames. The second specifies the pixel 
      format (so the leading 'mraw:' may be omitted) as one of: .mrgb555, 
      .mrgb565, .mrgb24, .mrgb32, .myuv24, .myuyv, .muyvy, .myuv444, .myuv422, 
      .myuv411, .myuv420, .myuv410, .myuv444p, .myuv422p, .myuv411p, .myuv420p, 
      .myuv410p. The optional third extension can be .gz or .bz2, in which case 
      the video file will be transparently decompressed from gzip or bzip2 
      compression, respectively. For example, a valid usage would be 
      "--in=somemovie.640x480.myuv420p.gz".
              --in=xmlfile:path/to/file.xml
                Reads input frames from an xml file which could contain 
      MetaData. This can be used to train the biasing algorithm or for object 
      recognition. The xml file can contain includes for other xml files. 
              --in=stimulus2D:path/to/file.stim2d
                Reads a text file created by a matlab script createStim.m and 
      generates input frames. createStim.m creates a text file which contains 
      1d time varying signals and the pixel location in an image they are 
      assigned. Useful for creating 2d step, impulse etc.  functions for 
      testing systems with both spatial and temporal dynamics. 
              --in=v4l
                NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM
                Reads input frames from a Video4Linux (V4L) framegrabber 
      device. There is no [spec] option here; the framegrabber can be 
      configured with ordinary command-line options which become available 
      after --in=v4l is selected.
              --in=v4l2
                Reads input frames from a Video4Linux version 2 (V4L2) 
      framegrabber device. There is no [spec] option here; the framegrabber can 
      be configured with ordinary command-line options which become available 
      after --in=v4l2 is selected.
              --in=ieee1394
                NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM
                Reads input frames from a Firewire camera using the libdc1394 
      library version 0.9 or 1.x. There is no [spec] option here; the camera 
      can be configured with ordinary command-line options which become 
      available after --in=ieee1394 is selected.
              --in=dc1394v2
                NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM
                Reads input frames from a Firewire camera using the libdc1394 
      library version 2.x. There is no [spec] option here; the camera can be 
      configured with ordinary command-line options which become available 
      after --in=dc1394v2 is selected.
              --in=qtgrab
                NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM
                Reads input frames from a QuickTime camera using Apple's 
      QuickTime library. There is no [spec] option here; the camera can be 
      configured with ordinary command-line options which become available 
      after --in=qtgrab is selected.
              --in=rtsp://URL
                NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM
                Reads input frames from an rtsp source using livemedia libs. 
      There is no [spec] option here; the camera can be configured with 
      ordinary command-line options which become available after --in=rtsp is 
      selected.
      
      GENERATED INPUT SOURCES
              --in=random[:WxH]
                Generates color input frames with R/G/B values drawn from a 
      uniform random distribution on [0,255]. The optional WxH spec can be used 
      to specify the dimensions of the generated images. The default image size 
      is 320x240.
              --in=dots[:WxH],MotionType
                Generates artificial greyscale (but in PixRGB format) moving 
      dots. [:WxH] indicate optional width and height of the stimulus The 
      default image size is 320x240. The motion type supported for now is: 
      Foe[flags] for Focus of Expansion (flags: bit 2: haveMotion , bit1: have 
      temporal gradient, bit 0: have spatial gradient)Planar for planar motion. 
      We are working on rotation, random, and single dot motion. In addition 
      user will be able to specify some of the critical parameters for each 
      motion type. 
              --in=bars[:WxH],MotionType
                Generates artificial greyscale (but in PixRGB format) bars. 
      [:WxH] indicate optional width and height of the stimulus The default 
      image size is 320x240. The motion type supported for now is: Normal for a 
      single vertical bar moving left to right, ApertureProblem for a 45deg bar 
      moving left to right . In the future the user will be able to specify 
      some parameters.for each motion type. 
              --in=shiftedImage:path/image:MotionType
                Generates moving color image stimuli The motion type supported 
      for now is: foe for Focus of Expansion, planar for planar motion. We are 
      working on rotation motionIn addition user will be able to specify some 
      of the critical parameters for each motion type. 
              --in=colorbars[:frameperiod]
                Generates a series of 'color bars' test images which include 
      bars of various colors, a set of grayscale bars, a frame counter, and a 
      clock. The optional [frameperiod] can be used to specify the desired 
      real-time inter-frame interval in any time units or Hz. Examples: 
      --in=colorbars:100ms will generate one test frame every 100ms; 
      --in=colorbars:4Hz will generate one test frame every 250ms (4Hz).
              --in=life[:WxH]
                Generates black+white input frames according to Conway's "Game 
      Of Life", a cellular automata system. The optional WxH spec can be used 
      to specify the dimensions of the generated images. The default image size 
      is 320x240.
              --in=World2D[:WxH]
               Generates a simple world composed of 2D objects like square, 
      lines, circles, and objects which are a combination of simpler objects. 
      The optional WxH spec can be used to specify the dimensions of the 
      generated images. The default image size is 256x256.
       
      INPUT FILTERS
              --in=bob:<childtype:childspec>
                This input type is a filter, whose [spec] is a nested type:spec 
      specification. The filter applies bob deinterlacing to frames coming from 
      the child input source. The bob deinterlacer forms deinterlaced images 
      alternately from the 'top' and 'bottom' half-fields of the original image 
      (corresponding to the even and odd-numbered scanlines). Each deinterlaced 
      image is formed by linearly interpolating to fill in the missing scan 
      lines. This type of deinterlacing generates twice as many frames, at 
      twice the frame rate, as the original input source. This is called 'bob' 
      deinterlacing because the resulting image sequence may appear to slightly 
      bob up and down as it alternates between interpolated top fields and 
      bottom fields. Examples: --in=bob:foo.mpg will deinterlace a movie; 
      --in=bob:v4l will deinterlace the input coming from a Video4Linux 
      framegrabber.
              --in=bhf:<childtype:childspec>
                This input type is a filter, whose [spec] is a nested type:spec 
      specification. The filter applies linear interpolation deinterlacing to 
      the bottom half-field of the original images.
              --in=thf:<childtype:childspec>
                This input type is a filter, whose [spec] is a nested type:spec 
      specification. The filter applies linear interpolation deinterlacing to 
      the top half-field of the original images.
              --in=buf:<childtype:childspec>
                This input type is a filter, whose [spec] is a nested type:spec 
      specification. This input type buffers the input frames of its child 
      input source into a frame queue using a background thread. Then, while 
      callers in the main thread read frames out from the buffered input 
      stream, the background thread continually tries to keep the queue filled. 
      Command-line parameters are available to control the buffer size 
      (--input-buffer-size), and also to control the desired behavior if the 
      queue underflows (--underflow-strategy), i.e., if somebody in the main 
      thread tries to read a frame while the queue is empty.
      

  --in-echo=<raster|display|mpeg|none> [none]  (std::string)
      Specify a destination for carbon-copies of the input frames. If more than 
      one --in-echo is given, then the copies are sent to each destination in 
      parallel.


Output Writing/Formatting Options:

  -R, --output-frames=[[first[-step]]-[last]]@[delay_or_rate] [0-1-MAX@30Hz]  
    (FrameRange)
      Output frame range and inter-frame delay or rate. See --input-frames for 
      details on the expected format. Variable rates can be specified as with 
      --input-frames, BUT NOTE that some output formats (e.g., movie encoders) 
      may not preserve the variable framerate information.

  --rescale-output=<width>x<height> [0x0]  (Dims)
      Rescale output frames to fixed dims, or 0x0 for no rescaling

  --[no]preserve-output-aspect [no]
      Preserve output frame aspect ratio if rescaling to fixed dims

  --zoom-output=<int> [0]  (int)
      Number of levels for output images to be zoomed larger (for positive 
      values) or smaller (for negative values); e.g. a zoom level of -3 would 
      reduce the image size by a factor of 8 in both width and height, while a 
      zoom level of +4 would increase the image size by a factor of 16 in both 
      width and height. This option is similar to --rescale-output, but is more 
      useful in the case where you just want to scale the output to half its 
      normal size, without having to know exactly what the normal size would 
      be. It is also useful in cases where you will be generating multiple 
      output streams with different natural sizes; in that case, 
      --rescale-output would rescale them all to the same size, while 
      --zoom-output lets you apply a uniform zoom to each of them so that their 
      relative sizes are maintained.

  --out=<[type]:[spec]> []  (std::string)
      Specify a destination for output frames. This option may be called 
      multiple times to build up an internal list of multiple parallel output 
      destinations. If --out=none is given, then any previous --out selections 
      are canceled. The general output paradigm is that the program may 
      generate multiple streams of output, and each of those streams may be 
      feed to one or more output destinations selected by --out options. For 
      example, the program might generate output streams named 'MapA' and 
      'MapB', and if --out=pnm is selected, then ultimately the generated files 
      will be named MapA000000.pnm, MapA000001.pnm, etc. and MapB000000.pnm, 
      MapB000001.pnm, etc.
      
      The general expected form of the argument to a --out option is 
      [type]:[spec], where [type] specifies the type of output stream (such as 
      raster files, movie files, onscreen windows, etc.), and [spec] provides 
      additional information for the output stream (such as a prefix for the 
      output file names). In certain common cases, the explicit [type] may be 
      omitted if it can be inferred from the filename dot-extension spec. Also, 
      for certain stream types, the [spec] is optional and may be omitted. 
      There are several general output stream types: (1) disk output 
      destinations, in which the output is saved in some form to one or more 
      files on disk; (2) GUI output destinations, in which the output images 
      are displayed in some type of GUI window(s); (3) text output 
      destinations, in which some information about the output images is 
      printed to the console; and (4) filters, which are output destinations 
      that merely transform the output on its way to a final destination (for 
      example, splitting an RGB image into its individual color components). 
      Finally, some output destinations may not be supported on every system, 
      depending on the operating system and available libraries; unavailable 
      output destinations are still included in the list below but are 
      specifically marked as being unavailable. Following is a list of possible 
      output stream types and the format expected of their [spec] options. In 
      the descriptions below, the phrase KEY refers to the output stream name 
      produced by the program (such as 'MapA' or 'MapB' in the previous 
      example), and which is NOT controllable from the command-line.
      
      DISK OUTPUT DESTINATIONS
              --out=raster:fileprefix
              --out=raster:path/to/fileprefix#nnn#
                Images will be saved to individual raster files on disk. The 
      format of the image files will be deterined by the --output-format option 
      (default=PNM). The filenames will consist of several parts, including the 
      fileprefix given in the [spec], the KEY provided by the program, the 
      frame number, and the filename extension appropriate to the image file 
      format, in this format: [fileprefix]-[KEY][framenumber][.ext]. The hyphen 
      between [fileprefix] and [KEY] is omitted if the fileprefix ends with a 
      '/' (that is, it just specifies a directory prefix). By default, the 
      frame number portion of the filename will be a 6-digit zero padded 
      number, but the length of zero padding can also be specified in the 
      fileprefix. If the fileprefix includes a pair of hashes ('#'), then the 
      characters between the hashes will be interpreted as a minimum field 
      width, and the frame number will be formatted as a string with 
      zero-padding to fill that minimum width. As special cases, a single '#' 
      is equivalent to '#6#', and '##' is equivalent to '#0#'. Note that the 
      location of this hash sequence within the fileprefix is irrelevant; the 
      frame number will always appear at the end of the filename, just before 
      the filename extension. For example, an unadorned --out=raster will 
      produce KEY000000.pnm, KEY000001.pnm, etc.; --out=raster:foo/bar will 
      produce foo/bar-KEY000000.pnm, foo/bar-KEY000001.pnm, etc; 
      --out=raster:foo/#2# will produce foo/KEY00.pnm, foo/KEY01.pnm, etc; 
      --out=raster:## will produce KEY0.pnm, KEY1.pnm. If the fileprefix 
      includes a directory component, then that directory must already exist.
              --out=pnm[:fileprefix]
              --out=pgm[:fileprefix]
              --out=ppm[:fileprefix]
              --out=pbm[:fileprefix]
                Like --out=raster with the image file format set to PNM 
      (Portable aNyMap). Despite the different option names here, the output 
      files generated will always have a '.pnm' extension.
              --out=pfm[:fileprefix]
                Like --out=raster with the image file format set to PFM, which 
      is our custom format to hold 32-bit floating-point values in a plain file 
      format similar to PNM. The filename extension will be '.pfm'. These files 
      can be read back in by various programs in the toolkit, and can be read 
      into matlab with pfmread.m. This file format encodes only grayscale 
      images, so color images will be converted to their luminance prior to 
      saving in PFM format.
              --out=png[:fileprefix]
                Like --out=raster with the image file format set to PNG 
      (Portable Network Graphics). The filename extension will be '.png'.
              --out=jpg/jpeg[:fileprefix]
                Like --out=raster with the image file format set to jpg. 
      Default quality is 75%.The filename extension will be '.jpg' or '.jpeg'.
              --out=rawvideo[:fileprefix]
                Like --out=raster with the image file format set to RAWVIDEO. 
      This is a raw file format in which the file contents include only the raw 
      image pixels; the file itself does not indicate the image dimensions or 
      the pixel type. Both of these pieces information are, however, encoded 
      into the filename extension, which will be of the form '.WWWxHHH.PIXFMT', 
      where WWW and HHH are the image width and height, and PIXFMT is one of 
      'rgb555', 'rgb565', 'rgb24', 'rgb32', 'yuv24', 'yuyv', 'uyvy', 'yuv444', 
      'yuv422', 'yuv411', 'yuv420', 'yuv410', 'yuv444p, 'yuv422p', 'yuv411p, 
      'yuv420p', or 'yuv410p', as appropriate. These files can be subsequently 
      read back in with the --in option.
              --out=txt[:fileprefix]
                Like --out=raster with the image file format set to TXT. The 
      filename extension will be '.txt'. This is a plain-text file format in 
      which image pixels are simply written as space-separated strings, with 
      one row of pixels per line of text. Color pixels are written as 
      individual R, G, B values. Thus, a line of pixels from a color image 
      would be encoded as R1 G1 B1 R2 G2 B2 R3 G3 B3, etc. Files generated in 
      this format can be read by matlab's load() function.
              --out=ccode[:fileprefix]
                Like --out=raster with the image file format set to CCODE. The 
      filename extension will be '.C'. In this format, images are encoded as 
      C++ source code suitable for pasting back into a source file in this 
      toolkit. The generated source will include one int variable for the image 
      width, one for the image height, and one C-style array containing the 
      image pixels in row-major format.
              --out=bkg-rawvideo:pfx1,pfx2,pfx3,...
                Similar to --out=rawvideo, but in this case the actual file 
      writing happens in a background thread. This is useful on multi-core 
      machines in cases where the main thread is fully occupied with some other 
      task or has other real-time constraints. The generated filenames are of 
      the form [pfx][KEY][NNNNNN][.ext], where [pfx] cycles through the list of 
      prefixes given in the [spec]. This allows output image files to be 
      striped across multiple directories and even multiple physical disks, 
      which can help to maximize usage of available disk bandwidth. For 
      example, assuming 640x480 YUV420P files are being saved with 
      --out=bkg-rawvideo:/disk1/a/,/disk2/b/, then the generated files will be 
      /disk1/a/KEY000000.640x480.yuv420p, /disk2/b/KEY000001.640x480.yuv420p, 
      /disk1/a/KEY000002.640x480.yuv420p, /disk2/b/KEY000003.640x480.yuv420p, 
      etc.
              --out=movie[:fileprefix]
                One movie file will be generated for each KEY, named as 
      [fileprefix][KEY][.ext]. The filename extension will be appropriate to 
      the movie codec selected with --output-codec. The video encoding is 
      handled by the ffmpeg library.
              --out=mpeg[:fileprefix]
                An alias for --out=movie.
              --out=ucbmpeg[:fileprefix]
                NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM
                Like --movie, but the video encoding is handled by ppmtompeg or 
      mpeg_encode (but neither of those was found on this system).
              --out=mgz[:fileprefix]
                Like --out=movie, but the output files will be saved in a 
      custom 'mgz' format. The format is essentially a gzip-compressed sequence 
      of raw images, all in a single file.
              --out=mraw[:fileprefix]
                Generates multi-frame raw files containing concatenated 
      rawvideo frames. The resulting files are completely identical to a 
      concatenation of the individual files generated with --out=rawvideo; 
      however, if you are going to be saving hundreds or thousands of frames, 
      then it may be more efficient (in terms of operating system and disk 
      overhead) to save them into a single file with --out=mraw than to save 
      into individual files with --out=rawvideo. Like --out=rawvideo, this 
      multi-frame format is a truly raw format with only pixel data in the file 
      itself; therefore, critical metadata (image dimensions and pixel format) 
      are encoded into the filename. The filename will have an extension of the 
      form '.WWWxHHH.mPIXFMT' (similar to the --out=rawvideo format, but note 
      the extra 'm' denoting "multi-frame"); WWW and HHH specify the image 
      dimensions and PIXFMT specifies the pixel format, as described for 
      --out=rawvideo. The multi-frame files generated by --out=mraw can be 
      subsequently read back in with a --in=mraw option. Note that although 
      --in=mraw can read gzip- or bzip2-compressed files, --out=mraw cannot 
      generate such files directly; rather, you can generate an an uncompressed 
      file with --out=mraw, then compress it separately with gzip or bzip2, and 
      then read the compressed file back in with --in=mraw.
      
      GUI OUTPUT DESTINATIONS
              --out=display
                Image files are sent to X11 windows on the display specified by 
      the $DISPLAY environment variable, or :0.0 if $DISPLAY is unset. One 
      window is generated for each KEY, and the KEY name is used as a window 
      title.
              --out=sdl
                NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM
                Image files are sent to an SDL window. This option can only be 
      used when there is exactly one KEY. The parameters (size, etc.) of the 
      SDL window can be controlled with other command-line parameters.
              --out=qt
                NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM
                Image files are sent to a Qt window. This window holds all KEY 
      streams using a tabbed interface: individual KEY streams can be selected 
      for viewing by clicking on their names in the listbox on the left side of 
      the window. The image display pane, on the right side of the window, 
      shows the image itself (at bottom), along with auxiliary information 
      about the image (at top), including its KEY name, a more detailed 
      description of the image (if available), its dimensions, and its 
      image/pixel format. The image display pane also includes a spinbox to 
      zoom the image in/out, as well as a button that can be used to save any 
      displayed image as a raster file. The main window also includes a pause 
      button that can be used to stop a simulation so that the various images 
      can reviewed.
      
      TEXT OUTPUT DESTINATIONS
              --out=hash[:filename]
                Write a simple checksum hash of each image to the given 
      filename. If the filename is omitted, or is given as '' (the empty 
      string) or '-' or 'stdout' or 'STDOUT', then the checksums will be 
      written to stdout; likewise, if the filename is 'stderr' or 'STDERR', 
      then the checksums will be written to stderr. In all other cases, the 
      checksums will be written to the named file.
              --out=info[:filename]
                At program shutdown time, write a summary of the number, size, 
      and type of frames written for each KEY. The summary is written to 
      stdout, stderr, or a named file, based on the given filename, in the same 
      way as described for --out=hash.
              --out=stats[:filename]
                Write a simple stats summary of output image, including the 
      min/mean/stdev/max values for grayscale images, and the 
      min/mean/stdev/max values for each R,G,B channel in color images. Also, 
      at program shutdown time, write an overall stats summary for each KEY of 
      all the images written for that KEY. The stats are written to stdout, 
      stderr, or a named file, based on the given filename, in the same ways as 
      described for --out=hash.
      
      OUTPUT FILTERS
              --out=null
                A special output destination that simply discards any frames it 
      receives. Note that this is different from --out=none: whereas --out=none 
      cancels any previous --out options, --out=null is just another output 
      destination that happens to do nothing.
              --out=splitrgb:destination
                An output filter that splits color images into their individual 
      R,G,B components, and then passes those components on to the destination 
      specified by the [spec]. The [spec] should be another output destination, 
      just as would normally be passed to --out. When the components are passed 
      to the next destination, their KEY names are altered to include '-r', 
      '-g', or '-b' suffixes, as appropriate. For example, --out=splitrgb:pnm 
      would split color images into their components and then save those 
      components to pnm files named KEY-r000000.pnm, KEY-g000000.pnm 
      KEY-b000000.pnm etc. Grayscale images passing through splitrgb will also 
      be "split" into components, but those components will just happen to be 
      identical.
              --out=luminance:destination
                An output filter that converts color images into their 
      luminance images, and then passes those luminance images on to the 
      destination specified by the [spec]. As with --out=splitrgb, the [spec] 
      is just another output destination, as would normally be passed to --out. 
      Color images passing through --out=luminance will have their KEY names 
      modified with a '-lum' suffix; grayscale images will pass through 
      unaltered, and their KEY names will remain unaltered as well.
              --out=colorize:destination
                An output filter that colorizes grayscale images with a 
      colormap. For now, the colormap is like matlab's 'jet' colormap and 
      cannot be changed. Grayscale images passing through --out=colorize will 
      have their KEY names modified with a '-colorized' suffix; images that are 
      already in color will pass through unaltered, and their KEY names will 
      remain unaltered as well.
              --out=coerce-grey:destination
              --out=coerce-rgb555:destination
              --out=coerce-rgb565:destination
              --out=coerce-rgb24:destination
              --out=coerce-rgb32:destination
              --out=coerce-yuv24:destination
              --out=coerce-yuyv:destination
              --out=coerce-uyvy:destination
              --out=coerce-yuv444:destination
              --out=coerce-yuv422:destination
              --out=coerce-yuv411:destination
              --out=coerce-yuv444p:destination
              --out=coerce-yuv422p:destination
              --out=coerce-yuv411p:destination
              --out=coerce-yuv420p:destination
              --out=coerce-yuv410p:destination
                Output filters that coerce images into a given rawvideo format, 
      if possible. Some coercions are atomic (implemented by a single 
      conversion function), while the remaining coercions are implemented by 
      the optimal sequence of atomic conversions. The optimal sequence for a 
      given conversion is the one that meets the following criteria, in order 
      of preference: (1) minimizes the number of lossy colorspace conversions, 
      like rgb->yuv, yuv->rgb, rgb->grey, or yuv->grey; (2) minimizes the 
      number of lossy spatial resolution conversions, like yuv444->yuv422 or 
      yuv444->yuv410p; (3) minimizes the number of lossy bit depth conversions, 
      like rgb24->rgb565 or rgb->555; (4) minimizes the total number of atomic 
      conversion steps.
      

  --[no]wait [no]
      Whether to wait for a keypress after each input/output frame

  --output-replicate=uint [1]  (unsigned int)
      How many times to replicate each output frame. This is useful if you want 
      to generate a '15fps' mpeg1 movie; mpeg1 doesn't support 15fps, but it 
      does support 30fps so if you use 30fps with output replication of 2, then 
      you will effectively end up with a 15fps movie.

  -+, --[no]keep-going [no]
      Keep going even after input is exhausted


What to Display/Save Options:

  --[no]display-boring [no]
      Display attention shifts to boring targets in green

  --foa-radius=<pixels> [0]  (int)
      Radius of the Focus Of Attention, or 0 to set it from input image dims

  --fovea-radius=<pixels> [0]  (int)
      Radius of the fovea, or 0 to set it from input image dims

  --[no]save-objmask [no]
      Save object mask

  --[no]display-interp-maps [yes]
      Use bilinear interpolation when rescaling saliency or other maps to 
      larger image sizes for display (this option does not affect the maps 
      themselves, just the way in which they are displayed)

  --display-map-factor=<factor> [0.0]  (float)
      When displaying/saving a saliency map, visual cortex output, task 
      relevance map, attention guidance map, etc, multiply the map by <factor> 
      to convert its raw values to values which can be displayed (typically, in 
      the [0..255] range). If the given <factor> is 0.0, then the map will be 
      normalized to [0..255] on a frame-by-frame basis (which may sometimes be 
      misleading as it may visually overemphasize maps which only contain very 
      low activation).

  --display-map=<SM|AGM|TRM|VCO> [AGM]  (std::string)
      Select which map to display in all displays which show a map alongside 
      the attention trajectory. The map values will be normalized according to 
      the value of --display-map-factor.

  --sv-type=<None|Std|Compress|EyeMvt|EyeMvt2|EyeRegion|EyeSimASAC|NerdCam|Stats|RecStats|Hand|EyeHand> [Std]  (std::string)
      Type of SimulationViewer to use. 
      	'Std' is the standard 2D viewer. 
      	'Compress' is a simple multi-foveated blurring viewer, in which a 
      selection of most salient locations will be represented crisply, while 
      the rest of the image will be increasingly blurred as we move away from 
      those hot spots. 
      	'EyeMvt' is a viewer for comparison between saliency maps and human eye 
      movements.
      	'EyeMvt2' is 'EyeMvt' plus it adds a concept of visual memory buffer.
      	'EyeMvtNeuro' is a viewer for comparing saliency maps and neural 
      responses.
      	'EyeRegion' is 'EyeMvt' plus it adds support for object 
      definitions/visualizations.
      	'EyeSim' simulates an eye-tracker recording from the model. 	We run 
      surprise control (ASAC) also from this command option. To use surprise 
      control use 'ASAC'.
       	'Stats' saves some simple statistics like mean and variance saliency, 
      location of peak saliency, etc.
      	'Hand' is a viewer for showing the combination of video andhuman hand 
      movement(joystick, etc).
      	'EyeHand' is a combination of EyeMvt and Hand.

  --[no]inverse-retinal [yes]
      If we are using a retinal type that performs a coordinate transfrom on 
      the input, should we undo that transform for display?

  --font-size=<uint> [10]  (unsigned int)
      Use the largest font available with width <= the value given here, or if 
      there is no such font, then use the smallest available font. Available 
      fonts range in width from 6 to 20.

  -T, --[no]save-trajectory [no]
      Save attention/eye/head trajectories

  -X, --[no]save-x-combo [no]
      Show combination trajec (left) + salmap (right)

  -Y, --[no]save-y-combo [no]
      Show combination trajec (top) + salmap (bottom)

  --[no]save-trm-x-combo [no]
      Show combination trajec (left) + salmap (middle)+  task-relevance-map 
      (right)

  --[no]save-trm-y-combo [no]
      Show combination trajec (top) + salmap (middle) +  task-relevance-map 
      (bottom) 

  --[no]save-trm-mega-combo [no]
      Show combination trajec (top left) + salmap (top right) +  
      task-relevance-map (bottom left) + AGM (bottom right) 

  -3, --[no]warp-salmap-3d [no]
      Show color image warped onto 3D salmap

  -K, --[no]mega-combo [no]
      Show trajectory, saliency and channels as a combo

  --mega-combo-zoom=<uint> [8]  (unsigned int)
      Zoom factor to use to display the --mega-combo conspicuity and feature 
      maps.

  --[no]mega-combo-topcm [yes]
      In --mega-combo displays, show only the top-level conspicuity maps, as 
      opposed to recursing through the channel hierarchy and showing all the 
      conspicuity maps, i.e., all output maps from all channels and 
      subchannels.

  -v, --crop-foa=<w>x<h> [0x0]  (Dims)
      Crop image to <w>x<h> around FOA location

  --[no]foveate-traj [no]
      foveate trajectory (cumulative if --display-additive)

  --[no]display-foa [yes]
      Display focus-of-attention as a circle

  --[no]display-patch [yes]
      Display small filled square at attended location

  --[no]display-foanum [no]
      Display attention shift number (0-based)

  --[no]display-traj [yes]
      Display attentional trajectory using red arrows

  --[no]display-eye [yes]
      Display small hollow square at eye position

  --[no]display-eye-traj [no]
      Display eye trajectory using red lines

  --[no]display-head [no]
      Display larger hollow square at head position

  --[no]display-head-traj [no]
      Display head trajectory using red lines

  --[no]display-time [yes]
      Display internal simulation time

  --[no]display-additive [yes]
      Display things additively

  --[no]display-highlight [no]
      Display highlight at focus-of-attention

  --[no]display-smmod [no]
      Display surprise-modulated image, using the saliency map (possibly 
      normalized using --display-map-factor) as contrast modulator

  --[no]display-larger-markers [no]
      Use larger drawings for FOA, FOV, and head markers than default

  -x, show-xwindow [OBSOLETE]
      This option is obsolete; if you want to see results in an onscreen window 
      you can try using --out=display or --out=qt instead


Attention Guidance Map Related Options:

  --[no]save-agm [no]
      Save attention guidance map

  --agm-type=<None|Std|Opt|NF|SC> [Std]  (std::string)
      Type of Attention Guidance Map to use. 'None' for no map (in which case 
      Brain will use the bottom-up saliency map for attention guidance), 'Std' 
      for the standard AGM that just computes the pointwise product between 
      bottom-up saliency map and top-down task-relevance map.  'NF' for maps 
      based on simple neural fields. 'SC' for an AGM based on the neural 
      architecture and functionality of the superior colliculus.


PrefrontalCortex-Related Options:

  --pfc-type=<Stub|OG|SB|GS> [Stub]  (std::string)
      Type of PrefrontalCortex to use. 'Stub' for a simple pass-through of 
      input (no biasing), 'OG' for the Optimal Gains pfc, 'SB' for SalBayes 
      Bayesian tuning of receptors, 'GS' for Guided Search. 


Retina-Related Options:

  --retina-type=<Stub|Std|CT> [Std]  (std::string)
      Type of Retina to use. 'Stub' for a simple pass-through of input frames, 
      'Std' for the standard retina that can foveate, shift inputs to eye 
      position, embed inputs within a larger framing image, foveate inputs, 
      etc. Use CT for a cortical or collicular transfrom (log-polar) of the 
      input image

  --[no]save-retina-input [no]
      Save retinal input images, with prefix RETIN-

  --[no]save-retina-output [no]
      Save retina output (possibly including foveation, shifting, embedding, 
      etc), with prefix RETOUT-

  --input-framing=<imagefile> []  (std::string)
      Filename of an image to used as a background into which input frames will 
      be embedded before processing. This image must be larger than the input 
      frames to be processed.

  --input-framing-pos=<i,j> [0,0]  (Point2D<int>)
      Position of the top-left corner of the input frames once embedded into 
      the larger background image specified by --input-framing, if any.

  --foveate-input-depth=<uint> [0]  (unsigned int)
      Depth of pyramid to use to foveate input frames

  --[no]shift-input [no]
      Shift input frames so that they are centered at current eye position

  --shift-input-bgcol=<r,g,b> [64,64,64]  (PixRGB<unsigned char>)
      Background color to use when shifting inputs using --shift-input

  --input-fov=<w>x<h> [0x0]  (Dims)
      If non-empty, centrally crop the input images to the given field-of-view 
      dimensions

  --[no]save-retina-pyr [no]
      Save pyramid used for retinal foveation, with prefix RET<level>-

  --retina-mask=<filename> []  (std::string)
      Mask retinal input images by a greyscale byte image of same dims, where 0 
      will transform a retinal pixel to black, 255 will not affect it, and 
      intermediate values will fade it to black.

  --[no]flip-input-horiz [no]
      Flip raw input images (before any embedding) horizontally.

  --[no]flip-input-vertic [no]
      Flip raw input images (before any embedding) vertically.


Inferior Temporal Cortex Options:

  --it-type=<None|Std|SalBayes|HMAX|CUDAHMAX> [None]  (std::string)
      Type of InferoTemporal to use. 'None','Std','SalBayes', 'HMAX' or 
      'CUDAHMAX'


General Brain/Attention-Related Options:

  --shape-estim-mode=<None|FeatureMap|ConspicuityMap|SaliencyMap|MTfeatureMap|ContourBoundaryMap|CSHistogram> [ConspicuityMap]  (ShapeEstimatorMode)
      Shape estimator mode

  --shape-estim-smoothmethod=<None|Gaussian|Chamfer> [None]  
    (ShapeEstimatorSmoothMethod)
      Shape estimator smooth method

  --ior-type=<None|Disc|ShapeEst> [ShapeEst]  (IORtype)
      Type of IOR to use; default is ShapeEstimator-based, but if no 
      ShapeEstimator information is available, then we fall back to Disc IOR

  --target-mask=<filename> []  (std::string)
      Name of a grayscale image file to be loaded and used as a targetmask for 
      Brain

  --pixperdeg=<float,float> [5.333333333,0.0]  (PixPerDeg)
      Pixels per degree of visual angle in the horizontal direction and 
      vertical directionif only one value is desired set the second to 0.0 
      (default).

  --[no]shape-estim-largeneigh [yes]
      Use a larger 3x3 neighborhood to track down a local maximum across scales 
      when true, otherwise use a 2x2 neighborhood.

  --clip-mask=<filename> []  (std::string)
      Name of a grayscale image file to be loaded and used as a clipmask for 
      Brain

  --rawinput-rect-border=<int> [128]  (int)
      Border size to use for the Retina's raw input rectangle (used to select 
      random samples in SimulationViewerCompress), in pixels.

  --[no]enable-pyramid-caches [yes]
      Whether to allow caching of commonly-needed image pyramids based on the 
      current input image, such as the intensity pyramid shared between the 
      intensity channel and the motion channels, or the laplacian pyramid 
      shared among the oriented gabor channels. There should be no reason to 
      disable pyramid caching except for debugging or profiling.

  --[no]blank-blink [no]
      Blank-out visual input during eye blinks

  --too-many-shifts=<int> [0]  (int)
      Exit after this number of covert attention shifts

  --too-many-shifts-per-frame=<int> [0]  (int)
      Wait untill the next frame after this number of covert attention shifts

  --boring-delay=<time> [200.0ms]  (SimTime)
      Attention shifts that come after an interval longer than this will be 
      considered 'boring' (for example, they may be represented differently in 
      output displays)

  --head-radius=<pixels> [50]  (int)
      Radius of the head position marker

  --multiretina-depth=<int> [5]  (int)
      Depth of pyramid used for foveation


Saliency Map Related Options:

  --boring-sm-mv=<float> [2.0]  (float)
      Saliency map level (in mV) below which attention shifts will be 
      considered 'boring' (for example, they may be represented differently in 
      output displays)

  --sm-type=<None|Std|StdOptim|Trivial|Fast> [Std]  (std::string)
      Type of Saliency Map to use. 'None' for no saliency map, 'Std' for the 
      standard saliency map using LeakyIntegrator neurons (takes a lot of CPU 
      cycles to evolve), 'Trivial' for just a non-evolving copy of the inputs, 
      'Fast' for a faster implementation thatjust takes a weighted average 
      between current and new saliency map at each time step.

  --[no]save-salmap [no]
      Save saliency map as an un-normalized float image, which is useful to 
      compare the absolute saliency values across several images or several 
      frames of a movie. You may use saliency/matlab/pfmread.m to read these 
      images into matlab, or saliency/bin/pfmtopgm to convert them to PGM 
      format.

  --[no]save-cumsalmap [no]
      Save cumulative saliency map so far as an unnormalized float image.

  --[no]salmap-sacsupp [yes]
      Use saccadic suppression in the saliency map

  --[no]salmap-blinksupp [no]
      Use blink suppression in the saliency map

  --salmap-maxwinv=<mV> [5.0]  (float)
      Winner voltage (in millivolts) which, if exceeded, will trigger global 
      inhibition over the saliency map. This helps avoiding that the saliency 
      map explodes if it receives constantly strong inputs.

  --salmap-iordecay=<0..1> [0.9999]  (float)
      Coefficient by which inhibition-of-return dies off at each time step


Task-Relevance Map Related Options:

  --trm-type=<None|Std|KillStatic|KillN|GistClassify|Tigs|Tigs2> [None]  
    (std::string)
      Type of Task-Relevance Map to use. 'None' for no map, 'Std' for the 
      standard TRM that needs an agent to compute the relevance of objects, 
      'KillStatic' for a TRM that progressively decreases the relevance of 
      static objects, and 'KillN' for a TRM that kills the last N objects that 
      have been passed to it, 'GistClassify' for classifythe current frame into 
      different categories and use the correspondingpre-defined TD map, 'Tigs' 
      for use the gist vector to get a top down mapTigs2 use the gist vector 
      combined with the PCA image vector to get atop down map


Attention Gate Related Options:

  --[no]save-ag [no]
      Save attention gate maps

  --ag-type=<Std|None> [None]  (std::string)
      Type of Attention Gate to use. Std or None


Winner-Take-All Related Options:

  --wta-type=<None|Std|StdOptim|Fast|Greedy|Notice> [Std]  (std::string)
      Type of Winner-Take-All to use. 'None' for no winner-take-all, 'Std' for 
      the standard winner-take-all using LeakyIntFire neurons (takes a lot of 
      CPU cycles to evolve), 'Fast' for a faster implementation that just 
      computes the location of the max at every time step, 'Greedy' is one that 
      returns, out of a number of possible targets above a threshold, the one 
      closest to current eye position. 'Notice' uses an adaptive leaky 
      integrate and fire neuron that trys to notice things across frames

  --[no]save-wta [no]
      Save winner-take-all membrane potential values

  --[no]wta-sacsupp [yes]
      Use saccadic suppression in the winner-take-all

  --[no]wta-blinksupp [yes]
      Use blink suppression in the winner-take-all


Eye/Head Saccade Controller Options:

  --initial-eyepos=<x,y> [-1,-1]  (Point2D<int>)
      Initial eye position, or (-1,-1) to start as undefined until the first 
      shift of attention, or (-2,-2) to start at the center of the image

  --ehc-type=<None|Simple|EyeTrack|Monkey> [None]  (std::string)
      Eye/Head Controller name (pick a name and then use --help to see options 
      specific to the chosen controller)


Hand Controller Options:

  --hand-type=<None|HandTrack> [None]  (std::string)
      Hand Controller name (pick a name and then use --help to see options 
      specific to the chosen controller)


Gist Estimator Options:

  --ge-type=<None|Std|FFT|Texton|CB|BBoF|SurfPMK|Gen> [None]  (std::string)
      Type of gist estimator to use


Visual Buffer Related Options:

  --vb-type=<Stub|Std> [Stub]  (std::string)
      Type of Visual Buffer top use. 'Stub' does nothing, and 'Std' is a 
      world-centered cumulative buffer that can be updated either continually 
      or at every attention shift and can make prediuctions about next saccade 
      target in world coordinates.


Channel-Related Options:

  --save-channel-outputs
      Save all channel outputs. EQUIVALENT TO: --save-featurecomb-maps 
      --save-conspic-maps

  --save-channel-internals
      Save all available channel internal maps. EQUIVALENT TO: --save-raw-maps 
      --save-feature-maps

  --best-multi-srs
      Use a tuned multi-spectral residual model. EQUIVALENT TO: 
      --vc-type=MultiSpectralResidual --maxnorm-type=Ignore 
      --noenable-pyramid-caches --multi-srs-sizes=/16,/8,/4,/2 
      --srs-output-blur-factor=0.055 --srs-spectral-blur=1 
      --srs-attenuation-width=0.0625 --map-combine-type=Max 
      --srs-lowpass-filter-width=2 

  --[no]use-older-version [no]
      Use the older version where we normalize responses within all feature 
      types

  --maxnorm-type=<None|Maxnorm|Fancy|FancyFast|FancyOne|FancyLandmark|Landmark|FancyWeak|Ignore|Surprise> [FancyOne]  (MaxNormType)
      Type of MaxNormalization to use

  --ori-interaction=<None|SubtractMean|c1,...,cn> [SubtractMean]  (std::string)
      This describes the way in which the different orientations within the 
      OrientationChannel interact with each other. 'None' is for no interaction 
      (default); for 'SubtractMean', the mean of all orientation pyramids is 
      subtracted from each orientation pyramid. You can determine your own 
      interaction by specifying a vector with interaction coefficients: 
      'c1,...,cn' where n is the number of orientations. c1 is the coeffiecient 
      for an orientation itself, c2 for the one with the next higher angle and 
      so on. For instance, for n=4, 'None' is the same as '1,0,0,0', and 
      'SubtractMean' is the same as '0.75,-0.25,-0.25,-0.25'

  --oricomp-type=<Steerable|Gabor|GaborEnergyNorm> [Steerable]  
    (OrientComputeType)
      Type of computation used to compute orientations

  --map-combine-type=<Sum|Max> [Sum]  (MapCombineType)
      Strategy used by ComplexChannel for combining output maps from 
      subchannels. Default strategy is summation; alternatives include 
      pixel-wise max.

  --[no]save-conspic-maps [no]
      Save conspicuity maps from all complex channels ("CO")

  --chanout-min=<float> [0.0]  (float)
      Min of the channel's output range

  --chanout-max=<float> [10.0]  (float)
      Max of the channel's output range

  --[no]use-split-cs [no]
      Use split positive/negative center-surround computations

  --levelspec=<cmin,cmax,delmin,delmax,maplev> [2,4,3,4,4]  (LevelSpec)
      LevelSpec to use in channels. This controls the range of spatial scales 
      used in constructing center-surround maps, and also controls the scale of 
      the channel output maps. cmin and cmax are the lowest (largest) and 
      highest (largest) pyramid levels to be used for the center scale in 
      center-surround operations. A level of 0 is the bottom pyramid level at 
      the original dimensions of the input image; each level above 0 is reduced 
      by a factor of 2 in the x and y dimensions, so e.g. level 4 is 16-fold 
      reduced in x and y. delmin and delmax represent the range of differences 
      at which the surround level is offset from the center level. maplev is 
      the scale at which channel output should be generated. For example, the 
      default setting of 2,4,3,4,4 will use center scales 2-4 with deltas of 3 
      and 4, for six center/surround pairs of 2/5, 2/6, 3/6, 3/7, 4/7 and 4/8, 
      and the channel output will be at scale 4.

  --qlen=<int> [1]  (int)
      Queue length for channels

  --qtime-decay=<float> [20.0]  (double)
      Time decay for channel queues

  --[no]save-raw-maps [no]
      Save raw input maps (pyramid levels) from all single channels ("SR")

  --[no]save-raw-maps-gist [no]
      Save raw input maps (pyramid levels) from all single channels for 
      gistcomputation  ("SR")

  --[no]save-feature-maps [no]
      Save center-surround feature maps from all single channels ("SF")

  --[no]save-featurecomb-maps [no]
      Save combined center-surround output maps from all single channels ("SO")

  --submap-algo=<Std|List> [Std]  (std::string)
      Name of the SubmapAlgorithm type to use in SingleChannel

  --[no]save-channel-stats [no]
      Save (append) several types of statistics  for each single channel to 
      sc_stats.txt

  --[no]save-stats-per-channel [no]
      When saving channel stats, should we put the data for each channel into 
      its own file?

  --[no]save-stats-per-channel-freq [no]
      When saving channel stats, should we also save frequency data?

  --save-channel-stats-name=<string> [sc_stats.txt]  (std::string)
      File name to use for single channel stats file

  --save-channel-stats-tag=<string> [NULL]  (std::string)
      Tag name to use for single channel stats file

  --gabor-intens=<double> [10.0]  (double)
      Intensity coefficient for Gabor channel

  --[no]use-trig-tab [no]
      Whether to accelerate trig operations by using table lookup, at the 
      expense of some loss of numerical precision

  --[no]direction-sqrt [no]
      Take square root of our Reichardt output if true

  --direction-lowthresh=<float> [3.0]  (float)
      Low threshold to allpy to eliminate small motion responses

  --color-comp-type=<Standard|Simple|StandardFull> [Standard]  
    (ColorComputeType)
      Type of computation used to compute RG and BY color opponencies

  --num-orient=<int> [4]  (unsigned int)
      Number of oriented channels

  --num-directions=<int> [4]  (unsigned int)
      Number of direction-selective motion channels


VisualCortex-Related Options:

  --vc-type=<None|Std|Beo|Surp|Int|Env|...> [Std]  (std::string)
      Type of VisualCortex to use:
        None: no VisualCortex at all
        Std: use standard (floating-point) channels, most flexible
        Beo: use Beowulf channels (requires Beowulf cluster)
        Surp: use Surprise channels, the fanciest to date
        Int: use integer-math channels, fast yet somewhat flexible
        Env: use super-fast integer-math channels, the fastest to date
        Entrop: entropy model, computing pixel entropy in 16x16 image patches
        EyeMvt: fake visual cortex built from human eye movement traces
        PN03contrast: Parkhurst & Niebur'03 contrast model
        Variance: local variance in 16x16 image patches
        Michelson: Michelson contrast as in Mannan, Ruddock & Wooding '96
        Tcorr: temporal correlation in image patches across frames
        Scorr: spatial correlation between image patches in a frame
        Info: DCT-based local information measure as in Itti et al, PAMI 1998
        SpectralResidual: Spectral residual cortex
        MultiSpectralResidual: Multi spectral residual cortex
      You may also configure which channels to use in your VisualCortex by 
      specifying a series of letters through the option --vc-chans. Finally, 
      you can apply modifiers by prefixing them to your vc type:
        Thread: is a modifier that works with Std, Surp, etc and
            is used to dispatch computations to worker threads; (note
            that to actually create the worker threads you must also give
            a '-j N' option to create N threads); it is OK to have fewer
            threads than channels, in which case each thread would simply
            perform more than one channel computation per input cycle).
            EXAMPLE: --vc-type=Thread:Surp

  --vc-chans=<CIOFM...> [CFIOM]  (std::string)
      Configure which channels to use in your VisualCortex by specifying a 
      series of letters from:
        C: double-opponent color center-surround
        D: dummy channel
        E: end-stop detector
        F: flicker center-surround
        G: multi-color band channel
        H: H2SV channel
        Y: Use a Depth Channel with spatial depth information
        I: intensity center-surround
        K: skin hue detector
        L: L-junction detector
        M: motion energy center-surround
        U: foe
        B: FoeMST
        N: intensity-band channel
        O: orientation contrast
        P: SIFT channel
        Q: CIELab Color channel
        R: Pedestrian channel
        S: composite single-opponent color center-surround
        T: T-junction detector
        V: short-range orientation interactions ("sox") channel
        W: contour channel
        X: X-junction detector
        Z: dummy zero channel
        A: Object Detection channel
        J: DKL Color channel
        U: Foreground Detection Channel
        i: Imagize 3-chanel Silicon Retina Channel
        s: Motion SpatioTemporal Energy Channel
        o: Motion Optical Flow Channel
      with possible optional weights (given just after the channel letter, with 
      a ':' in between). EXAMPLE: 'IO:5.0M' will use intensity (weight 1.0), 
      orientation (weight 5.0) and motion (weight 1.0) channels

  --vcx-outfac=<float> [1.0e-9]  (float)
      Factor applied to outputs of VisualCortex to scale them to Amps of 
      synaptic input currents to saliency map

  --vcx-noise=<float> [1.0e-12]  (float)
      Noise applied to outputs of VisualCortex after --vcx-outfac has been 
      applied

  --vcx-save-out-to=<filename.mgz> []  (std::string)
      Save the raw VisualCortex output map to the designated MGZ file if a 
      filename is specified. The saved outputs can later be re-read using the 
      --vcx-load-out-from option.

  --vcx-load-out-from=<filename.mgz> []  (std::string)
      Load the raw VisualCortex output map from the designated MGZ file if a 
      filename is specified. Typically, the MGZ file should contain maps which 
      have been previously saved using the --vcx-save-out-to option. CAUTION: 
      When this option is in use, we will not compute much in the VisualCortex, 
      instead reading the precomputed results off the disk. This means in 
      particular that whichever channels you may have in your VisualCortex will 
      not affect its output.

  --vcx-save-rawcs-out-to=<filename.mgz> []  (std::string)
      Save the raw center-surround maps for each channel to a single designated 
      MGZ file if a filename is specified. The maps will be concatinated 
      vertically. 

  --[no]save-vcx-output [no]
      Save output of visual cortex (=input to the saliency map) as a float 
      image in PFM format with absolute saliency values. This is good for 
      comparing several saliency maps obtained for different images. See 
      saliency/matlab/pfmreadmatlab.m for a program that reads PFM images into 
      Matlab.

  --[no]vcx-usemax [no]
      Use max across features instead of sum to yield the combined saliency 
      map.

  --vcx-weight-thresh=<float> [0.0]  (float)
      Lower threshold on total channel weight when counting the number of 
      non-zero weight channels to decide whether or not to apply one last round 
      of maxnorm to the VisualCortex output, if the number of non-zero-weight 
      channels is 2 or more.

  --vcx-rawcs-dims=<wxh> [40x30]  (Dims)
      The dimensions at which to save the raw center-surround maps. Rescaling 
      without interpolation will be performed if necessary. 


MBARI Related Options:

  --mbari-benthic-video
      Implements good choice of options to experiment with processing video 
      from a moving camera traversing the sea bottom. EQUIVALENT TO: 
      --levelspec=1-3,2-4,3 --num-orient=4 --mbari-save-original-frame-spec 
      --mbari-tracking-mode=Hough --mbari-mask-lasers=true 
      --mbari-saliency-input-image=Raw --mbari-min-event-frames=3 
      --sm-type=StdOptim --vc-chans=OIMF --use-random=true  --mbari-se-size=4 
      --rescale-input=960x540  --boring-sm-mv=0.25 
      --ori-interaction=SubtractMean --oricomp-type=Steerable 
      --shape-estim-smoothmethod=None --mbari-saliency-dist=5 
      --mbari-cache-size=30 --mbari-max-evolve-msec=1000 
      --mbari-keep-boring-WTA-points=true --maxnorm-type=FancyOne 
      --mbari-min-event-frames=1 --mbari-dynamic-mask=true 

  --mbari-eits-video
      Options used for processing Eye-in-the-Sea Video from the  Ocean Research 
      and Conservation Association (ORCA). EQUIVALENT TO: 
      --mbari-tracking-mode=KalmanFilter --mbari-keep-boring-WTA-points=yes 
      --mbari-save-boring-events=yes --mbari-save-original-frame-spec 
      --mbari-segment-algorithm-input-image=Luminance --mbari-color-space=RGB 
      --mbari-saliency-input-image=Raw --mbari-cache-size=2 
      --mbari-max-WTA-points=15 --mbari-max-evolve-msec=1000 --vc-chans=OIC 
      --use-random=true  --maxnorm-type=Maxnorm --oricomp-type=Steerable 
      --levelspec=1-3,2-5,3 --mbari-cache-size=2 --mbari-max-event-area=30000 
      --mbari-min-std-dev=10.0 --mbari-segment-algorithm=GraphCut 
      --mbari-event-expiration-frames=3 --rescale-input=320x240 
      --mbari-segment-graph-parameters=0.5,1500,500

  --mbari-toolsled-video
      Implements good choice of options to experiment with detecting  summary 
      of results from a camera mounted on a benthic toolsled with shadowing and 
      a moving frame. EQUIVALENT TO: --mbari-tracking-mode=KalmanFilterHough 
      --mbari-cache-size=60 --mbari-save-original-frame-spec --test-mode=true 
      --mbari-color-space=RGB --vc-chans=COKMF  --use-random=true 
      --mbari-dynamic-mask=true --maxnorm-type=FancyOne 
      --mbari-saliency-input-image=Raw --mbari-mask-lasers=true 
      --boring-sm-mv=0.25 --levelspec=1-3,2-4,1 --mbari-se-size=10 
      --qtime-decay=1.0 --mbari-max-evolve-msec=1000 
      --mbari-segment-algorithm=GraphCut  
      --mbari-segment-algorithm-input-image=DiffMean --mbari-saliency-dist=2 
      --shape-estim-smoothmethod=None --boring-sm-mv=1.0 
      --mbari-x-kalman-parameters=0.1,20.0 --mbari-y-kalman-parameters=0.1,20.0 
      

  --mbari-midwater-video
      Implements good choice of options to experiment with processing video 
      from a moving camera traversing the midwater sea column. EQUIVALENT TO: 
      --mbari-tracking-mode=KalmanFilterHough --mbari-cache-size=60 
      --mbari-segment-graph-parameters=0.90,50,30 
      --mbari-save-original-frame-spec --test-mode=true 
      --mbari-dynamic-mask=true --mbari-color-space=RGB --vc-chans=O  
      --use-random=true --maxnorm-type=FancyOne 
      --mbari-saliency-input-image=DiffMean --rescale-input=960x540 
      --contrast-enhance=true --levelspec=1-3,2-4,1 --mbari-se-size=10 
      --qtime-decay=1.0 --mbari-max-evolve-msec=500 
      --mbari-segment-algorithm=Best  
      --mbari-segment-algorithm-input-image=ContrastEnhance 
      --mbari-saliency-dist=5 --shape-estim-smoothmethod=None 
      --boring-sm-mv=1.0 --mbari-x-kalman-parameters=0.1,10.0 
      --mbari-y-kalman-parameters=0.1,10.0 

  --mbari-mosaic-stills
      Implements good choice of options to experiment with still frames 
      collected from a moving camera in mosaic form. EQUIVALENT TO: 
      --mbari-saliency-dist=1 --mbari-tracking-mode=None 
      --mbari-max-event-frames=1 --mbari-min-event-frames=1 
      --mbari-keep-boring-WTA-points=yes 
      --mbari-remove-overlapping-detections=False --boring-sm-mv=1.0 
      --levelspec=1-3,2-5,2 --shape-estim-smoothmethod=None 
      --mbari-save-boring-events=yes --maxnorm-type=FancyOne 
      --mbari-pad-events=75 --vc-chans=OIC --num-orient=8 --use-random=true 
      --mbari-rescale-saliency=960x540 --mbari-saliency-input-image=Raw 
      --mbari-max-WTA-points=50 --mbari-max-evolve-msec=5000

  --mbari-timelapse-stills
      Implements good choice of options to experiment with still frames 
      collected from a stationary time-lapse camera. EQUIVALENT TO: 
      --mbari-saliency-dist=1 --mbari-tracking-mode=NearestNeighbor 
      --mbari-keep-boring-WTA-points=yes --mbari-rescale-saliency=960x540 
      --mbari-save-boring-events=yes  
      --mbari-segment-algorithm-input-image=DiffMean 
      --mbari-color-space=RGB--mbari-saliency-input-image=Raw 
      --mbari-cache-size=10 --qtime-decay=1.0 --boring-sm-mv=1.0 
      --levelspec=1-3,2-5,2 --shape-estim-smoothmethod=None --vc-chans=OIC  
      --use-random=true --maxnorm-type=FancyOne --mbari-max-WTA-points=30 
      --mbari-max-evolve-msec=15000 

  --mbari-cache-size=<int> [30]  (int)
      The number of frames used to compute the running average

  --mbari-min-std-dev=<float> [0]  (float)
      Minimum std deviation of input image required for processing. This is 
      useful to remove black frames, or frames with high visual noise

  --mbari-load-events=fileName []  (std::string)
      Load the event structure from a text file instead of computing it from 
      the frames

  --mbari-load-properties=fileName []  (std::string)
      Load the event property vector from a text file

  --mbari-source-metadata=fileName []  (std::string)
      Add video input source information to XML output

  -, --[no]mbari-save-boring-events [no]
      Save boring events. Default is to remove boring (non-interesting) events, 
      set to true to save

  --mbari-save-events=fileName []  (std::string)
      Save the event structure to a text file

  --[no]mbari-save-event-features [no]
      Save the event features to .dat files. Used in classification.

  --mbari-save-event-num=ev1,ev1,...,evN; or: all []  (std::string)
      Save cropped, event-centered images of specific events, or all events. 
      Crops according to bounding box plus optional --mbari-event-pad.

  --[no]mbari-save-original-frame-spec [no]
      Save events in original frame size specs, but run saliency computation on 
      reduced frame size. This does nothing if the frames are not resized with 
      the --rescale-input option. Default is set to false

  --[no]mbari-save-output [no]
      Save output frames in MBARI programs

  --mbari-save-positions=fileName []  (std::string)
      Save the positions of events to a text file

  --mbari-save-properties=fileName []  (std::string)
      Save the event property vector to a text file

  --mbari-save-event-summary=fileName []  (std::string)
      Save a human readable summary of all the events to a text file

  --mbari-save-events-xml=fileName []  (std::string)
      Save a XML output per all events

  --mbari-pad-events=0-200 [0]  (int)
      Size to pad event-centered image

  --[no]mbari-display-results [no]
      Display algorithm output at various points in MBARI programs

  --[no]mbari-mark-candidate [yes]
      Mark candidates for interesting events in output frames of MBARI programs

  --[no]mbari-mark-foe [no]
      Mark the focus of expansion in the output frames of MBARI programs

  --mbari-mark-interesting=<None|Shape|Outline|BoundingBox> [BoundingBox]  
    (BitObjectDrawMode)
      Way to mark interesting events in output frames of MBARI programs

  --[no]mbari-mark-prediction [no]
      Mark the Kalman Filter's prediction for the location of an object in 
      output frames of MBARI programs

  --mbari-opacity=<0.0 ... 1.0> [1.0]  (float)
      Opacity of shape or outline markings of events

  --mbari-rescale-display=<width>x<height> [0x0]  (Dims)
      Rescale MBARI displays to <width>x<height>, or 0x0 for no rescaling

  --[no]mbari-save-results [no]
      Save algorithm results at various points in the MBARI programs to disk

  --[no]mbari-label-events [yes]
      Write event labels into the output frames in MBARI programs

  --[no]mbari-contrast-enhance [yes]
      Automatically contrast enhance gamma in input frames in MBARI programs

  --mbari-max-evolve-msec=<int> [500]  (int)
      Maximum amount of time in milliseconds to evolve the brain until stopping

  --mbari-max-WTA-points=<int> [20]  (int)
      Maximum number of winner-take-all points to find in each frame

  --[no]mbari-remove-overlapping-detections [yes]
      Do not remove overlapping detections

  --mbari-tracking-mode=<KalmanFilter|NearestNeighbor|Hough|NearestNeighborHough|KalmanFilterHough|None> [KalmanFilter]  (TrackingMode)
      Way to mark interesting events in output of MBARI programs

  --mbari-color-space=<RGB|YCBCR|Gray> [RGB]  (ColorSpaceType)
      Input image color space. Used to determine whether to compute saliency on 
      color channels or not

  --mbari-max-event-frames=<int> [-1]  (int)
      The maximum number of frames an event can be; defaults to indefinite

  --mbari-min-event-frames=<int> [1]  (int)
      The minimum number of frames an event must be to be candidate

  --mbari-max-event-area=<int> [0]  (int)
      The maximum area an event can be, to be candidate. When set to 0, 
      defaults to a multiplied factor of the foa size, which is derived from 
      the image size.

  --mbari-min-event-area=<int> [0]  (int)
      The minimum area an event must be to be candidate. When set to 0, 
      defaults to multiplied factor of the foa size, which is derived from the 
      image size.

  --mbari-saliency-dist=<int> [5]  (int)
      The number of frames to delay between saliency map computations 

  --mbari-mask-path=<file> []  (std::string)
      MaskPath: path to the mask image

  --mbari-bayes-net-path=<file> []  (std::string)
      Bayes network: path to the Bayes classifier network file

  --mbari-mask-xposition=<int> [1]  (int)
      MaskXPosition: x position of the mask point of reference; 

  --mbari-mask-yposition=<int> [1]  (int)
      MaskYPosition: y position of the mask point of reference; 

  --mbari-mask-width=<int> [1]  (int)
      MaskWidth: mask width; 

  --mbari-mask-height=<int> [1]  (int)
      MaskHeight: mask height; 

  --mbari-rescale-saliency=<width>x<height> [0x0]  (Dims)
      Rescale input to the saliency algorithm to <width>x<height>, or 0x0 for 
      no rescaling

  --mbari-segment-algorithm=<MeanAdaptive|MedianAdaptive|MeanMinMaxAdapative|GraphCut|Best> [Best]  (SegmentAlgorithmType)
      Segment algorithm to find foreground objects

  --mbari-segment-algorithm-input-image=<DiffMean|Luminance|ContrastEnhance> 
    [DiffMean]  (SegmentAlgorithmInputImageType)
      Segment algorithm input images type

  --mbari-segment-graph-parameters=sigma, k, minsize [0.75,500,50]  
    (std::string)
      Graph segment parameters, in the order sigma, k, minsize. Generally,the 
      defaults work.     Dont mess with this unless you need to.  See algorithm 
      details in Segmentation.C.

  --mbari-segment-adaptive-parameters=neighborhood, offset [20,7]  
    (std::string)
      Neighborhood size and size of the offset to subtract from the mean or 
      median in the segment algorithm

  --mbari-x-kalman-parameters=process noise, measurement noise [0.1,0.0]  
    (std::string)
      X direction Kalman filter parameters, in the order process noise, 
      measurement noise

  --mbari-y-kalman-parameters=process noise, measurement noise [0.1,0.0]  
    (std::string)
      Y direction Kalman filter parameters, in the order process noise, 
      measurement noise

  --mbari-se-size=1-20 [5]  (int)
      Size of structure element to do morhphological erode/dilate to clean-up 
      segmented image

  --mbari-saliency-input-image=<Raw|DiffMean|Max|RedGreenOpponent> [DiffMean]  
    (SaliencyInputImageType)
      Saliency input image type

  --mbari-bayes-net-features=<HOG3|HOG8|MBH3|MBH8|JET> [HOG8]  (FeatureType)
      Name of the features used in the Bayes classifier network file

  --[no]mbari-keep-boring-WTA-points [no]
      Keep boring WTA points from saliency computation. Turning this on will 
      increase the number of candidates, but can also increase thenumber of 
      false detections

  --[no]mbari-mask-lasers [no]
      Mask lasers commonly used for measurement in underwater video.

  --[no]mbari-dynamic-mask [no]
      Generate dyamic mask for brain during saliency computation using 
      segmented images 


Option Aliases and Shortcuts (may not always work):

  --top5
      Compute the saliency map and save the first five attended locations. This 
      will save five images, T000000.pnm to T000004.pnm which show the 
      trajectory taken by the first five shifts of the focus of attention, as 
      well as a text file 'top5.txt' with the coordinates of the attended 
      locations as well as other information.. EQUIVALENT TO: --nouse-fpe 
      --nouse-random -T --textlog=top5.txt --too-many-shifts=5 
      --output-frames=0-4@EVENT --out=raster: -+

  --just-initial-saliency-map
      Just save the initial saliency map (i.e., output from the VisualCortex) 
      before any shift of attention, and exit. The floating-point map will be 
      saved in our proprietary PFM image format and with prefix 'VCO', which is 
      harder to visualize but good for comparisons across images as it contains 
      absolute saliency values. See saliency/matlab/pfmreadmatlab.m for a 
      simple function to read a PFM image into Matlab. Note that the values in 
      the map are typically very small, as they are in Amperes of synaptic 
      current flowing into the integrate-and-fire neurons of the dynamic 
      saliency map. Typical values are in the nA range (1.0e-9 Amps). You can 
      also use saliency/bin/pfmtopgm to convert the map to a PGM image.. 
      EQUIVALENT TO: --nouse-fpe --nouse-random --save-vcx-output --out=raster: 
      --out=pfm: --too-much-time=0.1ms --output-frames=0-0@0.1ms 

  --movie
      Process in movie (multiframe) mode. EQUIVALENT TO: --nodisplay-traj 
      --nodisplay-additive --nouse-fpe --display-map-factor=50000 
      --display-map=AGM --nouse-random --nodisplay-foa --display-patch

  --moviefast
      Process in movie (multiframe) mode, making some approximations in the 
      computations to make processing fast albeit slightly different from the 
      gold standard. You still need to provide:
         --in=<movie>    input movie
         --out=display   or any other output
         -j <n>          for the number of parallel threads (default 4)
         -T              or similar option (like -X, -Y, -K) for what to 
      display.. EQUIVALENT TO: --movie --display-foa --sm-type=Fast 
      --wta-type=Fast --nodisplay-interp-maps --ior-type=None 
      --vc-type=Thread:Std --vc-chans=CIOFM -j 4 --timestep=30ms 
      --maxnorm-type=FancyWeak --input-frames=0-MAX@30ms 
      --output-frames=0-MAX@30ms --logverb=Error

  --moviefov
      Process in foveated movie (multiframe) mode with a ThreshFric 
      SaccadeController. EQUIVALENT TO: --movie --foveate-input-depth=6 
      --ehc-type=Simple --esc-type=Threshfric --ior-type=None 
      --trm-type=KillStatic

  --movieeyehead
      Process in foveated movie (multiframe) mode with a Monkey2 Eye/Head 
      SaccadeController and displays of head position. EQUIVALENT TO: 
      --moviefov --sc-type=Monkey2 --display-head

  --movieanim
      Process in foveated movie (multiframe) mode with a human-derived ModelW 
      Eye/Head SaccadeController, a WinnerTakeAllGreedy, no IOR, and displays 
      of head position. EQUIVALENT TO: --movie --foveate-input-depth=4 
      --sc-type=ModelW --ior-type=None --display-head --shape-estim-mode=None 
      --wta-type=Greedy --initial-eyepos=-2,-2 --initial-headpos=-2,-2

  --surprise
      Use standard surprise mode. EQUIVALENT TO: --vc-type=Surp 
      --vc-chans=CFIOM --maxnorm-type=Surprise --gabor-intens=20.0 
      --direction-sqrt --display-map=VCO --display-map-factor=1e11 
      --vcx-outfac=5.0e-9

  --surpriseRSVP
      Use RSVP optimized surprise mode. EQUIVALENT TO: --vc-type=Surp 
      --vc-chans=I:0.312778365O:1.566049997H:0.877309234 
      --maxnorm-type=Surprise --gabor-intens=20.0 --direction-sqrt 
      --display-map=VCO --display-map-factor=1e11 
      --vcx-outfac=5.0e-9--ior-type=None --nouse-random 
      --display-foa--sm-type=Trivial --wta-type=None--salmap-factor=1e12 
      --agm-factor=1e12

  --eyecompare
      Use standard mode for comparison between the model's output and a human 
      or monkey (or other) eye movement trace. Do not forget to set at least 
      the following depending on your specific experiment and data:
        --foa-radius=<radius>        depending on size of display;
        --eyetrack-data=<file, ...>  your .eyeS eye movement record file(s);
        --svem-out-fname=<file>      where the results will be saved;
        --svem-prior-distro=<file>   for non-uniform random sampling. 
      And very carefully check for other settings as well. EQUIVALENT TO: 
      --sv-type=EyeMvt --save-eyemvt-megacombo --wta-type=Std --ior-type=None 
      --shape-estim-mode=None --maxcache-size=0 --nowta-sacsupp 
      --nowta-blinksupp --nosalmap-sacsupp --nosalmap-blinksupp

  --eyedisplay
      Display one or more eye-tracking traces on top of a movie. This should be 
      used with the following options:
        --foa-radius=<radius>        depending on size of display;
        --eyetrack-data=<file, ...>  your .eyeS eye movement record file(s);
      And very carefully check for other settings as well. EQUIVALENT TO: 
      --sv-type=EyeMvt -T --vc-type=None --sm-type=None --trm-type=None 
      --wta-type=None --shape-estim-mode=None --fovea-radius=32 
      --maxcache-size=0 --delaycache-size=0

  --eyemap
      Build a cumulative eye movement density map from one or more eye movement 
      traces. This is useful if you have one or more observers that saw a same 
      stimulus and you want to build a spatial probability density function 
      from the eye traces. The resulting eye map can be compared, for example, 
      with saliency maps or can be sampled in specific target regions. This 
      should be used used with the following options:
        --in=<image>                  input image, for dims only;
        --vcem-eyefnames=<file, ...>  your .eyeS eye movement record file(s);
        --output-frames=0-0@XXms      set XX to the time when to save;
      And very carefully check for other settings as well. EQUIVALENT TO: 
      --nouse-fpe --nouse-random --save-vcx-output --out=raster: -+ 
      --vc-type=EyeMvt --sm-type=None --trm-type=None --wta-type=None 
      --shape-estim-mode=None --sv-type=None --vcem-forgetfac=1.0 
      --vcem-delay=0 --vcem-sigma=0 --vcem-saconly=true --vcem-usemax=true 
      --output-format=PFM

  --eyehand-display
      Display one or more eye-tracking and hand movement traces on top of a 
      movie. This should be used with the following options:
        --foa-radius=<radius>        depending on size of display;
        --eyetrack-data=<file, ...>  your .eyeS eye movement record file(s);
        --handtrack-data=<file, ...> your .hanD hand movement record file(s);
      And very carefully check for other settings as well. EQUIVALENT TO: 
      --sv-type=EyeHand -T --vc-type=None --sm-type=None --trm-type=None 
      --wta-type=None --shape-estim-mode=None --fovea-radius=32 
      --maxcache-size=0 --delaycache-size=0

  --train-optigains
      Extract salience information to train the optimal gain biasing model of 
      Navalpakkam and Itti (IEEE-CVPR 2006; Neuron 2007). This will essentially 
      select multi-band features and setup the training process. You will then 
      also need to provide:
        --in=xmlfile:file.xml   input image and region of interest metadata;
        --stsd-filename=file.pmap  to save saliency of target/distractors;
      The resulting sT/sD pmap files can be combined together using 
      app-combineOptimalGains to yield a gains file that can be used with the 
      --guided-search option. EQUIVALENT TO: --vc-type=Std --vc-chans=GNO 
      --nouse-older-version --out=display --pfc-type=OG --num-colorband=6 
      --num-satband=6 --num-intensityband=6 --num-orient=8

  --guided-search
      Use a set of gains to bias salience computation in a manner similar to 
      Jeremy Wolfe's Guided Search (Wolfe, 1994). The gains are read from a 
      ParamMap file which can be obtained, for example, by using 
      --train-optigains. You will need to set the following:
        --in=image.png              input image to process;
        --gains-filename=file.pmap  file to load the gains from. EQUIVALENT TO: 
      --vc-type=Std --vc-chans=GNO --nouse-older-version -K --out=display 
      --pfc-type=GS --num-colorband=6 --num-satband=6 --num-intensityband=6 
      --num-orient=8

  --kinect-demo
      Demonstrate the Kinect sensor by computing saliency over a combination of 
      RGB and Depth images.. EQUIVALENT TO: -K --in=kinect --out=display 
      --maxnorm-type=Maxnorm --retina-type=Stub --nodisplay-interp-maps 
      --ior-type=None --vc-type=Thread:Std -j 6 --vc-chans=YCIOFM
