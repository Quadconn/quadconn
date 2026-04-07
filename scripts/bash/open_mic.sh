TARGET_PORT="${1:-3004}"
AUDIO_DEVICE="${2:-hw:0}"
TARGET_HOST="${3:-100.119.158.85}"
OPUS_BITRATE="64000"

echo "Starting audio stream to $TARGET_HOST:$TARGET_PORT..."

# pipeline exec
exec gst-launch-1.0 -v \
    alsasrc device="$AUDIO_DEVICE" ! \
    audioconvert ! \
    audioresample ! \
    "audio/x-raw,rate=48000,channels=1" ! \
    opusenc bitrate="$OPUS_BITRATE" ! \
    rtpopuspay ! \
    udpsink host="$TARGET_HOST" port="$TARGET_PORT" "$@"