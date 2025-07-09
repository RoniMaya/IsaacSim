# ffmpeg properties for fastAPI stream


ServerUtils.start_fastapi()
ffmpeg_process = subprocess.Popen([
    'ffmpeg',
    # Input Flags
    '-f', 'rawvideo',
    '-pix_fmt', 'rgb24',        # The pixel format of the data from Numpy
    '-s', f'{WIDTH}x{HEIGHT}',  # Input resolution
    '-r', str(RENDERING_FREQUENCY), # Match the FFmpeg input rate to our render frequency
    '-i', '-',                  # Input from stdin pipe

    # Output Flags
    '-c:v', 'libx264',          # Codec
    '-preset', 'ultrafast',     # Encoding speed vs. compression
    '-tune', 'zerolatency',     # CRITICAL: Optimizes for real-time streaming
    '-pix_fmt', 'yuv420p',      # IMPORTANT: Most web players require this pixel format.
    '-bufsize', '1000k',        # Video buffer size
    '-g', str(RENDERING_FREQUENCY * 2), # Group of Pictures (GOP) size (e.g., a keyframe every 2 seconds)
    '-f', 'mpegts',             # Muxer format for streaming
    '-'                         # Output to stdout pipe
], stdin=subprocess.PIPE, stdout=subprocess.PIPE)



# different ffmpeg properties for H.264 stream (with different buffsize)

ffmpeg_process = subprocess.Popen([
    'ffmpeg',
    '-f', 'rawvideo',
    '-pix_fmt', 'rgb24',
    '-s', f'{WIDTH}x{HEIGHT}',
    '-r', str(RENDERING_FREQUENCY),
    '-i', '-',

    '-c:v', 'h264_nvenc',
    '-preset', 'p1',
    '-tune', 'll',
    '-b:v', '50k',                          # ADD: Set a constant bitrate (e.g., 2 Mbps)
    '-bufsize', '50k',                      # ADD: Set a buffer size (often same as bitrate)
    '-g', str(RENDERING_FREQUENCY * 2),    # ADD: Set a keyframe interval, same as your working command

    '-pix_fmt', 'yuv420p',
    '-f', 'rtsp',
    'rtsp://localhost:8554/mystream'
], stdin=subprocess.PIPE)

