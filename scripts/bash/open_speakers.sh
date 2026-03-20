
# specify port, then audio device defaults
# ./open_speakers.sh 5000 hw:3
LISTEN_PORT="${1:-3005}"
AUDIO_DEVICE="${2:-hw:1}"

echo "Listening for audio stream on UDP port $LISTEN_PORT..."
echo "Outputting to audio device: $AUDIO_DEVICE"

# 3. Execute the pipeline
# We use backslashes (\) to break the pipeline into readable lines
exec gst-launch-1.0 -v \
    udpsrc port="$LISTEN_PORT" ! \
    "application/x-rtp,media=audio,clock-rate=48000,encoding-name=OPUS,payload=96" ! \
    rtpopusdepay ! \
    opusdec ! \
    audioconvert ! \
    audioresample ! \
    alsasink device="$AUDIO_DEVICE" "$@"