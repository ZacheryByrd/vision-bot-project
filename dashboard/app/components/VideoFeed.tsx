"use client";

const VIDEO_STREAM_URL =
  process.env.NEXT_PUBLIC_VIDEO_STREAM_URL ||
  "http://localhost:8080/stream?topic=/vision_bot/debug_image";

/**
 * Renders the live MJPEG stream from web_video_server. A plain <img>
 * tag works because MJPEG-over-HTTP is just a sequence of JPEG frames
 * in one long-lived response -- no client-side decoding needed.
 */
export default function VideoFeed({ connected }: { connected: boolean }) {
  return (
    <div className="video-feed">
      {connected ? (
        // eslint-disable-next-line @next/next/no-img-element
        <img src={VIDEO_STREAM_URL} alt="Live camera feed from vision_bot" />
      ) : (
        <div className="video-feed__placeholder">
          <span className="video-feed__pulse" />
          Waiting for connection to the robot...
        </div>
      )}
    </div>
  );
}
