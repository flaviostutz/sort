#!/bin/sh

echo "Starting cam-event-detector..."
cam-event-detector \
  --loglevel="$LOG_LEVEL" \
  --cam-id="$CAM_ID" \
  --video-source-url="$VIDEO_SOURCE_URL" \
  --event-post-endpoint="$EVENT_POST_ENDPOINT" \
  --event-object-image-enable="$EVENT_OBJECT_IMAGE_ENABLE" \
  --event-scene-image-enable="$EVENT_SCENE_IMAGE_ENABLE" \
  --event-max-keypoints="$EVENT_MAX_KEYPOINTS"
