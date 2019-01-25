# rosbag2video

    rosbag2video.py
    rosbag to video file conversion tool 
    by Maximilian Laiacker 2016
    post@mlaiacker.de
## install:

avconv is needed and can be installed on Ubuntu with:

    sudo apt install libav-tools



## usage:
rosbag2video.py [--fps 25] [--rate 1] [-o outputfile] [-s (show video)] [-t topic] bagfile1 [bagfile2] ...

    rosbag2video converts image sequence(s) in ros bag file(s) to video file(s) with fixed frame rate using avconv.
    avconv needs to be installed!
    If no output file (-o) is given the filename '<topic>.mp4' is used and default output codec is h264.
    Multiple image topics are supported only when -o option is _not_ used.
    avconv will guess the format according to given file extension.
    Compressed and raw image messages are supportet with mono8 and bgr8/rgb8.
    
    -t topic
    only the images from topic "topic" are used for the video output

## example output:

    >rosbag2video.py -r 50 -o test.mp4 peng_2015-12-14-13-*.bag
    using  50.0  FPS
    /cam_peng/color_rect/image_raw/compressed  with datatype: sensor_msgs/CompressedImage
    ############# USING ######################
    avconv version 9.18-6:9.18-0ubuntu0.14.04.1, Copyright (c) 2000-2014 the Libav developers
      built on Mar 16 2015 13:20:58 with gcc 4.8 (Ubuntu 4.8.2-19ubuntu1)
    [mjpeg @ 0x8a92ee0] Estimating duration from bitrate, this may be inaccurate
    Input #0, mjpeg, from 'pipe:':
      Duration: N/A, bitrate: N/A
        Stream #0.0: Video: mjpeg, yuvj420p, 1620x1220 [PAR 1:1 DAR 81:61], 50 fps, 50 tbr, 50 tbn
    [libx264 @ 0x8aa27e0] using SAR=1/1
    [libx264 @ 0x8aa27e0] using cpu capabilities: MMX2 SSE2Fast SSSE3 SSE4.2
    [libx264 @ 0x8aa27e0] profile High, level 4.2
    [libx264 @ 0x8aa27e0] 264 - core 142 r2389 956c8d8 - H.264/MPEG-4 AVC codec - Copyleft 2003-2014 - http://www.videolan.org/x264.html - options: cabac=1 ref=3 deblock=1:0:0 analyse=0x3:0x113 me=hex subme=7 psy=1 psy_rd=1.00:0.00 mixed_ref=1 me_range=16 chroma_me=1 trellis=1 8x8dct=1 cqm=0 deadzone=21,11 fast_pskip=1 chroma_qp_offset=-2 threads=3 lookahead_threads=1 sliced_threads=0 nr=0 decimate=1 interlaced=0 bluray_compat=0 constrained_intra=0 bframes=3 b_pyramid=2 b_adapt=1 b_bias=0 direct=1 weightb=1 open_gop=0 weightp=2 keyint=250 keyint_min=25 scenecut=40 intra_refresh=0 rc_lookahead=40 rc=crf mbtree=1 crf=23.0 qcomp=0.60 qpmin=0 qpmax=69 qpstep=4 ip_ratio=1.25 aq=1:1.00
    Output #0, mp4, to 'test.mp4':
      Metadata:
        encoder         : Lavf54.20.4
        Stream #0.0: Video: libx264, yuvj420p, 1620x1220 [PAR 1:1 DAR 81:61], q=-1--1, 50 tbn, 50 tbc
    Stream mapping:
      Stream #0:0 -> #0:0 (mjpeg -> libx264)
    Press ctrl-c to stop encoding
    frame=   44 fps=  4 q=0.0 Lsize=     365kB time=0.84 bitrate=3558.5kbits/s    s/s    
